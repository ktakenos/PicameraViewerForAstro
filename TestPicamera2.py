#!/usr/bin/env python3

# -*- coding: utf-8 -*-

"""
TestPicamera2.py - ダブルバッファリングによる連続RAW撮影テスト・脚本.
露光時間4秒間、ダブルバッファ buffering による最小化待ち時間・ms単位のレポート表示.

(中略… docstring は省略)
"""

# --- メタデータ出力用設定 ──────────────────────────────────────
import sys
DEBUG_MODE = "--debug" in sys.argv

import json

import os

os.environ["LIBCAMERA_LOG_LEVELS"] = "INFO" if DEBUG_MODE else "FATAL"

import time
import queue
import threading
import cv2
import numpy as np

from picamera2 import Picamera2
N_FRAMES        = 10                          # 総撮影コマ数
EXP_TIME_US     = 4_000_000                   # 露光時間 (マイクロ秒) = 4秒
ANALOG_GAIN     = 16.0                        # アナログゲイン

# ホワイトバランスのゲイン値（相対比）
# ※ GeminiMade版と同等の明るさにするため、R/B ゲインを追加する。
#    AWB/AE を完全にOFFにして手動制御する場合、RAWストリームには内部ゲインが
#    含まれないため、ここで明示的に補正する必要がある。
WB_R        = 1.2                         # 赤 (R) — GeminiMade版と一致
WB_G        = 1.0                         # 緑 (G) 
WB_B        = 1.5                         # 青 (B) — GeminiMade版と一致

# ── IMX477 センサー定数（デフォルト値、実行時にmetadataで上書き可能）──────
# BlackLevel: IMX477のセンサー固有の黒レベルオフセット（12bit ADC値）。
#   RAW値からこれを差し引かないと、暗部が不要に持ち上がりダイナミックレンジ圧縮の原因になる。
#   ※ 実際の値は Picamera2 metadata の "SensorBlackLevels" または "BlackLevelLegacy" から取得可能
IMX477_BLACKLEVEL = 512.0                # IMX477/12bit のBlackLevel (ADC単位, デフォルト)

# WhiteLevel (SAT): IMX477/12bit のADC飽和レベルは理論上 = 2^12 - 1 = 4095。
#   ※ rpicam-still DNG と同等にするには、BlackLevel引き算を行わずに
#     raw × (65535.0 / WL) で単純な線形スケールを用いる方が正確になる。
IMX477_WHITELVL = 4095.0                # IMX477/12bit のADCフルスケール値 (65535/4095=16.0 の係数)

# ── Bayerパターン判定用フラグ ───────────────────────────────
# True → BayerRG (rggb: (0,0)=R), False → BayerBG (bggr: (0,0)=B)
# ※ 実際の値は実行時の sensor_format / ColourFilterArray で検出する（デフォルト=自動判定）
BAUER_PATTERN_DETECTED = None           # "RGGB", "BGGR", "GRBG", "GBRG" のいずれかに設定

def _detect_bayer_pattern(sensor_fmt: str, metadata: dict) -> str:
    """sensor_format名 + ColourFilterArray メタデータからBayerパターンを推測する.
    
    libcamera/Picamera2のフォーマット命名ルール:
      - SBGGR12 → BayerBG (0,0)=Blue
      - SRGGB12 → BayerRG (0,0)=Red  
      - SGBRG12 → unspecified / sensor固有
    
    ColourFilterArray メタデータ（存在する場合）の信頼度が高い.
      "bggr" → BGGR, "rggb" → RGGB, "grbg" → GRBG, "gbgr" → GBRG
    """
    # ① ColourFilterArray (metadata) を優先して判定
    cfa = None
    for key in ("ColourFilterArray", "CFA"):
        if key in metadata:
            cfa = metadata[key]
            break
    if isinstance(cfa, str):
        cfa_upper = cfa.upper()
        if "BGGR" in cfa_upper:
            return "BGGR"
        elif "RGGB" in cfa_upper:
            return "RGGB"
        elif "GRBG" in cfa_upper:
            return "GRBG"
        elif "GBRG" in cfa_upper:
            return "GBRG"

    # ② sensor_format からフォールバック判定
    sf = sensor_fmt.upper()
    if "BGGR" in sf:
        return "BGGR"
    elif "RGGB" in sf:
        return "RGGB"
    elif "GRBG" in sf:
        return "GRBG"
    elif "GBRG" in sf:
        return "GBRG"
    
    # ③ 最終フォールバック: IMX477 は一般的に SBGGR12 (BGGR)
    print(f"   ⚠️ WARN: Bayerパターン自動判定不可. センサー形式={sensor_fmt}. BGGR を仮定.")
    return "BGGR"


def _extract_black_level(metadata: dict, default_bl: float = IMX477_BLACKLEVEL) -> float:
    """metadata から BlackLevel を抽出する.
    
    SensorBlackLevels は dict ({'r': x, 'g': y, 'b': z}) または単一値の形式がある.
    平均してスカラー値を返す.
    """
    bl_key = None
    for k in ("SensorBlackLevels", "BlackLevelLegacy"):
        if k in metadata:
            bl_key = k
            break
    
    raw_val = metadata.get(bl_key) if bl_key else None
    if raw_val is None:
        return default_bl
    
    if isinstance(raw_val, (int, float)):
        return float(raw_val)
    
    if isinstance(raw_val, dict):
        vals = [v for v in raw_val.values() if isinstance(v, (int, float))]
        if vals:
            return sum(vals) / len(vals)
    
    return default_bl


def _extract_saturation_level(metadata: dict, default_wl: float = IMX477_WHITELVL) -> float:
    """metadata から SaturationLevel を抽出する. 
    """
    for k in ("SaturationLevel",):
        if k in metadata:
            val = metadata[k]
            if isinstance(val, (int, float)):
                return float(val)
    return default_wl


def _compute_scale(bl: float, wl: float):
    """BlackLevel / WhiteLevel から 12bit→16bit スケール係数を計算."""
    effective = wl - bl
    if effective <= 0:
        print(f"   ⚠️ WARN: 有効なダイナミックレンジが{effective}以下. デフォルトスケールを使用します.")
        return 65535.0 / 4095.0
    return 65535.0 / effective

RAW_SIZE_W       = 4056                  # RAWデータの幅 (ピクセル数)
RAW_SIZE_H       = 3040                  # RAWデータの高さ (ピクセル数)

# キューサイズ: ダブルバッファリング 
QUEUE_SIZE      = 2

# --- 共有状態-----------------------------------------------------------
# 注記: _shared_lock は proc_start_times のみを保護する。
#   - proc_start_times : メインスレッドと後処理スレッドの両方からアクセスされるためロックで保護必须。
#   - cap_intervals_ms : メインスレッドのみが操作すること設計されており、ロックは不要。
cap_intervals_ms = []                     # 各フレームごとのキャプチャ所要時間 (ms) [メインスレッド専有]
sensor_timestamps = []                   # SensorTimestamp蓄積リスト [メインスレッド専有]
proc_start_times = []                      # 後処理開始時刻 (各フレームごと) [_shared_lock保護下]
_shared_lock    = threading.Lock()         # proc_start_times へのスレッドセーフなアクセス用

# --- サーミナル色装飾 -----------------------------------------------------------
def bold(msg):            return f"\033[1m{msg}\033[0m"
def underline(msg):       return f"\033[4m{msg}\033[0m"

# --- RAW 12bit unpacking (Packed uint8 → Unpacked uint16) -----------------------
def _unpack_raw_2BPP(packed_uint8: np.ndarray, raw_h: int, raw_w: int, pq_alternating: bool = False) -> np.ndarray:
    """SBGGR12 / 2画素=3バイトパック形式を、行単位stride/padding対応でuint16に展開.

    [技術的背景]
    IMX477 (および同様の Bayer 12bit センサ) の lossless-packed Raw は、行横方向に
    2画素 (各12bit) を 3バイトに圧縮している。パターン:

        Byte0         Byte1              Byte2
      | p0[7:0] | {p0[11:8]|p1[3:0]} | p1[11:4] |

    libcamera は行末にアラインメント・パディングバイトを追加するため、本関数は
    各行の有効データのみをスライスし、余分なpaddingを破棄してからunpackする。

    [P/Q パターンについて]
      pq_alternating=True を指定すると、トリプレットごとの交替パターン (偶数→P / 奇数→Q) を適用する。
      Qパターンのバイト順序: | p0[7:0] | {p1[3:0]|p0[11:8]} | p1[11:4] |
      デフォルト False で pure P unpacking (既存仕様との完全互換).

    Parameters
    ----------
    packed_uint8 : np.ndarray, dtype=uint8
        shape=(H, stride) の生データ. stride = (raw_w*3//2) + padding_bytes.
    raw_h : int
        RAW画像の高さ (ピクセル).
    raw_w : int
        RAW画像の幅 (ピクセル) — unpack 後の値.
    pq_alternating : bool, default=False
        True の場合 P/Q 交替unpackingを適用する。

    Returns
    -------
    np.ndarray, shape=(raw_h, raw_w), dtype=uint16
        復元済みの物理 RAW 値 (各画素 0〜4095).
    """
    # 各行の有効packedバイト数: 2画素→3バイト, raw_w は偶数前提
    expected_bytes_per_row = (raw_w // 2) * 3

    result = np.empty((raw_h, raw_w), dtype=np.uint16)

    for r in range(raw_h):
        # 行末paddingカット (stride > effective bytes の場合がある)
        row_valid = packed_uint8[r, :expected_bytes_per_row].astype(np.uint16)

        n_triplets = len(row_valid) // 3
        triples  = row_valid[:n_triplets * 3].reshape((n_triplets, 3))
        b0 = triples[:, 0]
        b1 = triples[:, 1]
        b2 = triples[:, 2]

        # Pパターン unpack: p0[7:0] | ((b1&0xF0)<<4) / (b2<<4)|(b1&0x0F)
        px0_p = b0 | ((b1 & 0xF0).astype(np.uint16) << 4)
        px1_p = (b2.astype(np.uint16) << 4) | (b1 & 0x0F)

        if pq_alternating:
            # Qパターン unpack (バイト順序入れ替え):
            #   b0=p0[7:0], b1={(p1[3:0])|(p0[11:8]<<4)}, b2=p1[11:4]
            px0_q = b0 | ((b1 & 0x0F).astype(np.uint16) << 8)
            px1_q = (b2.astype(np.uint16) << 4) | (b1 >> 4)

            # P/Q 交替: 偶数トリプレット→P, 奇数トリプレット→Q
            trip_is_p = np.arange(n_triplets) % 2 == 0

            row_pixels = np.empty(n_triplets * 2, dtype=np.uint16)
            for t in range(n_triplets):
                if trip_is_p[t]:
                    row_pixels[t*2]   = px0_p[t]
                    row_pixels[t*2+1] = px1_p[t]
                else:
                    row_pixels[t*2]   = px0_q[t]
                    row_pixels[t*2+1] = px1_q[t]
        else:
            # pure P unpacking (既定動作、既存コードとの互換)
            row_pixels = np.empty(n_triplets * 2, dtype=np.uint16)
            row_pixels[0::2] = px0_p
            row_pixels[1::2] = px1_p

        result[r, :] = row_pixels

    # 検証: max > 4095 なら警告
    if np.max(result) > 4095:
        print(f"   ⚠️  WARN: Unpacked RAW max={np.max(result)} > 4095 (12bit越え). "
              f"パッキング形式が想定と異なる可能性があります.")

    return result


# --- デモザイク処理 (numpy版、uint16精度を保証) -----------------------------
def _demosaic_bgrg(raw16: np.ndarray) -> np.ndarray:
    """uint16 BayerBG デモザイク → uint16 RGB (numpyのみで実装).
    
    OpenCV の cv2.cvtColor はバージョンにより内部的な型変換を行い、
    16bit→8bit にダウンキャストする場合がある。この関数は numpy による
    バイリニア補間で16bit精度を完全に保証する.
    
    BayerBG パターン:
        B   Gw | Gb  R
        Gb  R  | ...
      (0,0)→B / (0,1)→Gw / (1,0)→Gb / (1,1)→R
    
    Parameters
    ----------
    raw16 : np.ndarray, shape=(H, W), dtype=uint16
        WB補正後の Bayer 生データ.
    
    Returns
    -------
    np.ndarray, shape=(H, W, 3), dtype=uint16
        RGB形式の画像 (R/G/B各チャンネルとも uint16).
    """
    h, w = raw16.shape
    src = raw16.astype(np.float32)
    
    # --- 各色成分を配置 (BayerBG: (0,0)→B / (0,1)→Gw / (1,0)→Gb / (1,1)→R) ---
    R = np.zeros((h, w), dtype=np.float32)
    B = np.zeros((h, w), dtype=np.float32)
    Gw = np.zeros((h, w), dtype=np.float32)
    Gb = np.zeros((h, w), dtype=np.float32)

    R[1::2, 1::2]      = src[1::2, 1::2]
    B[0::2, 0::2]      = src[0::2, 0::2]
    Gw[0::2, 1::2]     = src[0::2, 1::2]
    Gb[1::2, 0::2]     = src[1::2, 0::2]

    # --- np.pad で境界を拡張し、スライシング形状問題を一括回避する -----------
    def _bilinear_interpolate(channel):
        """上下左右の既知ピクセルからバイリニア補間 (np.uint16互換)."""
        padded = np.pad(channel, pad_width=1, mode='edge')
        up   = padded[0:-2, 1:-1]
        dn   = padded[2:,   1:-1]
        left = padded[1:-1, 0:-2]
        right= padded[1:-1, 2:]
        return (up + dn + left + right) / 4.0

    R  = _bilinear_interpolate(R)
    B  = _bilinear_interpolate(B)
    Gw = _bilinear_interpolate(Gw)
    Gb = _bilinear_interpolate(Gb)

    # --- RGBにスタックし uint16 に戻す (G = (Gw+Gb)/2 ) -------------------
    G = 0.5 * (Gw + Gb)
    rgb = np.clip(np.stack([R, G, B], axis=2), 0, 65535).astype(np.uint16)

    return rgb



def _demosaic_rggb(raw16: np.ndarray) -> np.ndarray:
    """uint16 BayerRG (CFA=rggb) デモザイク → uint16 RGB.
    
    IMX477の実際のカラーフィルタアレイ配列パターンは rggb であり、
    (0,0)=赤 / (0,1)=緑w / (1,0)=緑b / (1,1)=青。
    
    Parameters
    ----------
    raw16 : np.ndarray, shape=(H, W), dtype=uint16
        WB補正後の BayerRG 生データ.
    
    Returns
    -------
    np.ndarray, shape=(H, W, 3), dtype=uint16
        RGB形式の画像 (R/G/B各チャンネルとも uint16).
    """
    h, w = raw16.shape
    src = raw16.astype(np.float32)
    
    # --- BayerRG 配置: (0,0)→R / (0,1)→Gw / (1,0)→Gb / (1,1)→B ---
    R  = np.zeros((h, w), dtype=np.float32)
    B  = np.zeros((h, w), dtype=np.float32)
    Gw = np.zeros((h, w), dtype=np.float32)
    Gb = np.zeros((h, w), dtype=np.float32)
    
    R[0::2, 0::2]      = src[0::2, 0::2]   # 赤は偶数行・偶数列
    B[1::2, 1::2]      = src[1::2, 1::2]   # 青は奇数行・奇数列
    Gw[0::2, 1::2]     = src[0::2, 1::2]   # 緑wは偶数行・奇数列
    Gb[1::2, 0::2]     = src[1::2, 0::2]   # 緑bは奇数行・偶数列
    
    # --- np.pad で境界を拡張し、スライシング形状問題を一括回避 ------------
    def _bilinear_interpolate(channel):
        """上下左右の既知ピクセルからバイリニア補間 (np.uint16互換)."""
        padded = np.pad(channel, pad_width=1, mode='edge')
        up   = padded[0:-2, 1:-1]
        dn   = padded[2:,   1:-1]
        left = padded[1:-1, 0:-2]
        right= padded[1:-1, 2:]
        return (up + dn + left + right) / 4.0
    
    R  = _bilinear_interpolate(R)
    B  = _bilinear_interpolate(B)
    Gw = _bilinear_interpolate(Gw)
    Gb = _bilinear_interpolate(Gb)
    
    # --- RGBにスタックし uint16 に戻す (G = (Gw+Gb)/2 ) -------------------
    G = 0.5 * (Gw + Gb)
    rgb = np.clip(np.stack([R, G, B], axis=2), 0, 65535).astype(np.uint16)
    
    return rgb


# BGGR = BayerBG と同じ配置なのでエイリアスにする
_demosaic_bggr = _demosaic_bgrg


def _demosaic_grgb_fallback(raw16: np.ndarray, pattern: str) -> np.ndarray:
    """GRBG/GBRG Bayerパターンのフォールバック用デモザイク.
    
    GRBG:  (0,0)=Gw / (0,1)=R  / (1,0)=B  / (1,1)=Gb
    GBRG:  (0,0)=Gw / (0,1)=B  / (1,0)=R  / (1,1)=Gb
    
    RGGB デモザイクを流用し、R/B チャンネルを入れ替える.
    
    Parameters
    ----------
    raw16 : np.ndarray, shape=(H, W), dtype=uint16
        WB補正後の BayerRAW 生データ.
    pattern : str
        "GRBG" または "GBRG".
    
    Returns
    -------
    np.ndarray, shape=(H, W, 3), dtype=uint16
        RGB形式の画像.
    """
    h, w = raw16.shape
    src = raw16.astype(np.float32)
    
    if pattern == "GRBG":
        # (0,0)=Gw / (0,1)=R / (1,0)=B / (1,1)=Gb
        R  = np.zeros((h, w), dtype=np.float32)
        B  = np.zeros((h, w), dtype=np.float32)
        Gw = np.zeros((h, w), dtype=np.float32)
        Gb = np.zeros((h, w), dtype=np.float32)
        
        R[0::2, 1::2]     = src[0::2, 1::2]   # R は偶数行奇数列
        B[1::2, 0::2]     = src[1::2, 0::2]   # B は奇数行偶数列
        Gw[0::2, 0::2]    = src[0::2, 0::2]   # Gw は偶数行偶数列
        Gb[1::2, 1::2]    = src[1::2, 1::2]   # Gb は奇数行奇数列
    
    else:  # GBRG
        # (0,0)=Gw / (0,1)=B / (1,0)=R / (1,1)=Gb
        R  = np.zeros((h, w), dtype=np.float32)
        B  = np.zeros((h, w), dtype=np.float32)
        Gw = np.zeros((h, w), dtype=np.float32)
        Gb = np.zeros((h, w), dtype=np.float32)
        
        R[1::2, 0::2]     = src[1::2, 0::2]   # R は奇数行偶数列
        B[0::2, 1::2]     = src[0::2, 1::2]   # B は偶数行奇数列
        Gw[0::2, 0::2]    = src[0::2, 0::2]   # Gw は偶数行偶数列
        Gb[1::2, 1::2]    = src[1::2, 1::2]   # Gb は奇数行奇数列
    
    def _bilinear_interpolate(channel):
        padded = np.pad(channel, pad_width=1, mode='edge')
        up   = padded[0:-2, 1:-1]
        dn   = padded[2:,   1:-1]
        left = padded[1:-1, 0:-2]
        right= padded[1:-1, 2:]
        return (up + dn + left + right) / 4.0
    
    R  = _bilinear_interpolate(R)
    B  = _bilinear_interpolate(B)
    Gw = _bilinear_interpolate(Gw)
    Gb = _bilinear_interpolate(Gb)
    
    G = 0.5 * (Gw + Gb)
    rgb = np.clip(np.stack([R, G, B], axis=2), 0, 65535).astype(np.uint16)
    
    return rgb
def _post_process(q, done_event):
    """後処理ワーカー: キューからRAWを取り出し、WB→デモザイク→TIFF化.
    
    [キューアイテム形式]
      (raw_data, cap_time_ms, frame_idx, black_level, saturation_level, bayer_pattern)
    
    [注意]
    - 各フレームが完了したら done_event.set() でメインスレッドへ通知.
    - capture_request() ブロック時間中に前フレームを処理するパイプライン動作.
    """
    try:
        while True:            
            item = q.get()           # キュー待ち / ポイズンピル (None) が入るまでブロック

            if item is None:
                break                  # 終端シグナル受信→終了
            
            raw_data, cap_time_ms, _frame_idx, black_level, saturation_level, bayer_pattern = item

            # --- 後処理開始 -----------------------------------------------------
            
            ts_proc_start = time.monotonic() * 1000
            with _shared_lock:
                proc_start_times.append(ts_proc_start)
            
            output_filename = f"capture_raw_{str(_frame_idx).zfill(4)}.tiff"

            try:
                # ── SBGGR12 は Packed uint8 (2BPP) のため .view(np.uint16) で解除 ──
                # GeminiMade版と同様にパック解除し、width方向のpaddingをクロップ
                rd16 = raw_data.view(np.uint16)[:, :RAW_SIZE_W].copy()

                # ── CFE(BYR2)は既に16bitスケーリング済みのため、追加の倍率変換なし ──
                # CFEがセンサーの12bit RAWをBYR2(16bit)に展開済みなので、×16.0 の
                #   追加ステップを行わない (GeminiMade版と同様).
                rb = rd16.astype(np.float32)

                # WB補正 (Bayer パターンに応じた配置で適用) — ゲインは1.0なので実質恒等変換
                if bayer_pattern == "BGGR":
                    rb[0::2, 0::2] *= WB_B    # B
                    rb[0::2, 1::2] *= WB_G    # G
                    rb[1::2, 0::2] *= WB_G    # G
                    rb[1::2, 1::2] *= WB_R    # R
                elif bayer_pattern == "RGGB":
                    rb[0::2, 0::2] *= WB_R    # R
                    rb[0::2, 1::2] *= WB_G    # G
                    rb[1::2, 0::2] *= WB_G    # G
                    rb[1::2, 1::2] *= WB_B    # B
                elif bayer_pattern == "GRBG":
                    rb[0::2, 0::2] *= WB_G
                    rb[0::2, 1::2] *= WB_R
                    rb[1::2, 0::2] *= WB_B
                    rb[1::2, 1::2] *= WB_G
                elif bayer_pattern == "GBRG":
                    rb[0::2, 0::2] *= WB_G
                    rb[0::2, 1::2] *= WB_B
                    rb[1::2, 0::2] *= WB_R
                    rb[1::2, 1::2] *= WB_G
                else:
                    rb[0::2, 0::2] *= WB_R
                    rb[0::2, 1::2] *= WB_G
                    rb[1::2, 0::2] *= WB_G
                    rb[1::2, 1::2] *= WB_B

                # スケール不要(CFEが既に16bit化済み)。clipのみ。
                rc = np.clip(rb, 0, 65535).astype(np.uint16)

                print(f"   [unpack] dtype={rd16.dtype}, range=[{int(rd16.min())}, {int(rd16.max())}], mean={rd16.mean():.1f} (shape={rd16.shape})")
                print(f"   [scale] CFE(BYR2)既スケール済み→WBゲイン({WB_R}/{WB_G}/{WB_B})のみ適用")

                # デモザイク処理 → RGB カラースペース (cv2.cvtColor で GeminiMade版と同等に)
                if bayer_pattern == "BGGR":
                    img_rgb = cv2.cvtColor(rc, cv2.COLOR_BayerBG2RGB)
                elif bayer_pattern == "RGGB":
                    img_rgb = cv2.cvtColor(rc, cv2.COLOR_BayerRG2RGB)
                elif bayer_pattern == "GRBG":
                    img_rgb = cv2.cvtColor(rc, cv2.COLOR_BayerGR2RGB)
                elif bayer_pattern == "GBRG":
                    img_rgb = cv2.cvtColor(rc, cv2.COLOR_BayerGB2RGB)
                else:
                    # バックアップとして BGGR を使用
                    img_rgb = cv2.cvtColor(rc, cv2.COLOR_BayerBG2RGB)

                # ※ OpenCVのcvtColor(BayerXX2RGB)はR,G,B順に出力するが、
                #   cv2.imwrite は内部で常に B-G-R として扱うため、
                #   RGB→BGRの変換を適用して R/B チャンネル順を正しくする。
                img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)

                # cv2.imwrite: dtypeがuint16なので自動で16ビットTIFFとなる
                cv2.imwrite(output_filename, img_bgr)

            finally:
                ts_proc_end   = time.monotonic() * 1000
                proc_time_ms  = round(ts_proc_end - ts_proc_start, 2)
                print(f"- [後処理時間] [{output_filename}]: {proc_time_ms:.2f} ms")
                done_event.set()

    except Exception as e:
        import traceback; traceback.print_exc()

# --- main関数 --------------------------------------------------------------
def main(): 
    print(bold(f": 撮影テスト [{__file__}] 実行開始"))

    # メッセージの視覚的に区切る
    div_msg = "\n" + "=" * 32 + " "

    print(div_msg + f"<設定>" )
    print(f" - 露光時間: {EXP_TIME_US / 1e3} 秒 [{EXP_TIME_US} マイクロ秒]") 
    print(f" - アナログゲイン: {ANALOG_GAIN}")
    print(f" - キューのサイズ (ダブルバッファリング): {QUEUE_SIZE}\n")
    
    # 現在時刻の取得 (可視化・統計情報用)
    ts_start = time.monotonic() * 1000

    # 次に追加する: キューとイベントオブジェクト
    
    print(div_msg + "<キュー & イベント初期化>" )

    picam_cap_queue = queue.Queue(maxsize=QUEUE_SIZE)
    
    post_proc_event = threading.Event()
    post_proc_event.clear()  # 未処理状態（クリア）

    # 次に追加する: Picamera2 の設定・起動
    
    print(div_msg + "<カメラ初期化>" )
    
    # カメラの制御インスタンスを作成
    picam2 = Picamera2()
    
    camera_stopped = False  # stop() の多重呼出しを防止するためのフラグ

    # --- カメラのストリーム・構成 ---------------------------------
    config_main = {"format": "XRGB8888", "size": (64, 64)}
                                            # メインストリームは低解像度で十分（RAWのみが重要）

    # --- RAWストリームは SBGGR12 で固定（GeminiMade版と同等）--------------
    # ※ SensorFormat(packing=None) は Picamera2 の内部バグを誘発するため、
    #   パックされた SBGGR12 (uint8, 2BPP形式) を直接使用し、
    #   後処理側で .view(np.uint16) によるパック解除を行う。
    config_raw  = {"format": "SBGGR12",     "size": (RAW_SIZE_W, RAW_SIZE_H)}
    
    try:
        config = picam2.create_still_configuration(main=config_main,
                                                   raw   =config_raw)
        
        picam2.configure(config)      # 構成を適用

        # --- [修正#1] AE/AWB無効化 + 手動露出設定を start() 前に実施 ---
        # set_controls() は「次のリクエスト」に適用されるため、start() より前にセットする。
        # ※ AWBも同時に無効化しないと内部ゲイン補正が干渉する可能性がある。
        print(div_msg + "<カメラ・コントロール設定>")
        try:
            picam2.set_controls({
                "ExposureTime": EXP_TIME_US,
                "AnalogueGain": ANALOG_GAIN,           # 手動でのゲイン指定
                "AeEnable": False,                     # 自動露出を停止
                "AwbEnable": False,                    # 自動ホワイトバランスも停止（ゲイン干渉回避）
            })

            print(f"   - [完了] 露出時間: {EXP_TIME_US}μs")
            print(f"         アナログゲイン     : {ANALOG_GAIN}")
            print("       自動露出モードを無効化")
            print("       自動ホワイトバランスを無効化")
        except Exception as ex:
            print(div_msg + f"<エラー>: カメラ制御パラメータの設定に失敗しました.\n{ex}")
            camera_stopped = True
            sys.exit(-1)

        picam2.start()               # カメラに命令により動作開始

        print(f"   - [完了] カメラの初期設定と起動 ({ts_start} ms)")

        # --- RAWストリームの可用性を検証 (Picamera2 API確認済み: stream_map["raw"]) ---
        if not getattr(picam2, 'stream_map', {}).get('raw'):
            print(div_msg + "<エラー>: RAWストリームが有効になっていません。"
                  " Raspberry Pi カメラの接続を確認してください.")
            camera_stopped = True
            picam2.stop()
            sys.exit(-1)

    except Exception as ex: 
        print(div_msg + f"<エラー>: Picamera2の初期化中に問題が発生しました.\n{ex}")

        # すべての処理を早期に終了
        camera_stopped = True
        sys.exit(-1)

    
    # ── [追加] プレキャプチャでセンサーパラメータ・Bayerパターンを取得 ──────────
    print(div_msg + "<プレキャプチャによるセンサー情報取得>" )
    
    try:
        preview_req = picam2.capture_request()
        try:
            preview_meta = preview_req.get_metadata()
            
            # センサーフォーム名を記録
            sensor_fmt_name = str(picam2.sensor_format)
            print(f"   - センサー形式     : {sensor_fmt_name}")
            
            # Bayerパターン判定 (ColourFilterArray + フォーム名から)
            global BAUER_PATTERN_DETECTED
            BAUER_PATTERN_DETECTED = _detect_bayer_pattern(sensor_fmt_name, preview_meta)
            print(f"   - Bayerパターン    : {BAUER_PATTERN_DETECTED}")
            
            # BlackLevel / SaturationLevel のデフォルト取得 (1フレーム目のみログ出力)
            p_bl  = _extract_black_level(preview_meta)
            p_sat = _extract_saturation_level(preview_meta)
            print(f"   - BlackLevel (参照): {p_bl:.0f}")
            print(f"   - SaturationLevel  : {p_sat:.0f}")

            # --debug が有効なら、プレキャプチャで取得可能な全メタデータキーを出力
            if DEBUG_MODE:
                # キーのみ一覧表示
                meta_keys = sorted(preview_meta.keys())
                print(f"   - [DEBUG] メタデータキー数: {len(meta_keys)}")
                print(f"   - [DEBUG] SensorBlackLevels raw: {preview_meta.get('SensorBlackLevels', 'N/A')}")
                print(f"           SaturationLevel raw  : {preview_meta.get('SaturationLevel', 'N/A')}")
                # ゲイン関連メタデータ確認
                for gk in ("AnalogueGain", "DigitalGain", "AwbGain", "ColourGains"):
                    print(f"           {gk}             : {preview_meta.get(gk, 'N/A')}")
                # 自動制御状態確認
                for ek in ("AeEnabled", "AwbEnabled"):
                    print(f"           {ek}                 : {preview_meta.get(ek, 'N/A')}")
            
        finally:
            preview_req.release()
    
    except Exception as ex:
        print(div_msg + f"<警告>: プレキャプチャ中に想定外の値が発生した. メタデータフォールバックを使用.\n{ex}")
        # フォールバックは _detect_bayer_pattern 内で BGGR に設定される
    
    # 設定値は既に start()前に設定済み (上記参照)

    # --- 後処理スレッドを開始する (別スレッドでアクティブにする) ---------------
    
    print(div_msg + "<後処理スレッドスタート>" )

    proc_thread = threading.Thread(target=_post_process,
                                   args=(picam_cap_queue, post_proc_event),
                                   daemon=True)
    
    # タスクをバックグラウンドで実行する
    proc_thread.start()

    time.sleep(0.5)  # 種々の準備を待つ（カメラの安定化など）

    ## 先にスレッドが正常に開始したことを確認
    if not proc_thread.is_alive(): 
        print(div_msg + "<警告>: 後処理スレッドが起動しなかった.\n"
              f"  スクリプトを停止します.")
        camera_stopped = True
        picam2.stop()
        sys.exit(-1)

    
    # --- <キャプチャ・ループ> 開始 --------------------------------------------
    
    print(div_msg + "非同期撮影開始 (RAWデータ取得 & バッファ送信)")

    ts_frame_start   = time.monotonic() * 1000

    try:

        for frame_idx in range(1, N_FRAMES+1):
            
            # 環境をリセット
            #   - 次のフレームの後処理完了を待つイベントの初期化
            post_proc_event.clear() 

            print(f"\n-  [{frame_idx}/{N_FRAMES}] 非同期撮影開始: {time.strftime('%X')}.")
            
            start_time_ms = ts_frame_start    
                                           # 撮影開始時刻 (ms)
           
            try:
                # capture_request() でリクエストを取得 (メタデータ付き)
                request = picam2.capture_request()

                try:
                    metadata = request.get_metadata()

                    # --- 实际控制値 + オート制御状態のログ ────────────────
                    actual_exp_us = metadata.get("ExposureTime", "N/A")
                    actual_analog_gain = metadata.get("AnalogueGain", "N/A")
                    actual_digital_gain = metadata.get("DigitalGain", "N/A")
                    actual_awb_gain_r = metadata.get("AwbGain", {}).get("Red", "N/A") if isinstance(metadata.get("AwbGain"), dict) else metadata.get("AwbGain", "N/A")
                    actual_awb_gain_b = metadata.get("AwbGain", {}).get("Blue", "N/A") if isinstance(metadata.get("AwbGain"), dict) else "N/A"
                    ae_enabled        = metadata.get("AeEnabled", "N/A")
                    awb_enabled       = metadata.get("AwbEnabled", "N/A")   # v2 API key
                    ts_sensor         = metadata.get("SensorTimestamp", "N/A")

                    # 設定露光時間との整合性チェック（±0.5%許容）
                    if isinstance(actual_exp_us, (int, float)):
                        rel_err = abs(actual_exp_us - EXP_TIME_US) / EXP_TIME_US * 100
                        exp_ok = "[OK]" if rel_err <= 0.5 else f"[MISMATCH {rel_err:.2f}%]"
                    else:
                        exp_ok = "[?]"

                    # ゲイン整合性チェック（±5%許容）
                    if isinstance(actual_analog_gain, (int, float)):
                        gain_rel_err = abs(float(actual_analog_gain) - ANALOG_GAIN) / ANALOG_GAIN * 100
                        gain_ok = "[OK]" if gain_rel_err <= 5.0 else f"[MISMATCH {gain_rel_err:.2f}%]"
                    else:
                        gain_ok = "[?]"

                    # 1フレーム目のみゲイン詳細ログ（重複出力抑制）
                    if frame_idx == 1:
                        print(f"   - [GAIN] 設定AnalogueGain={ANALOG_GAIN}, 实际控制AnalogueGain={actual_analog_gain} {gain_ok}")
                        print(f"           DigitalGain={actual_digital_gain}")
                        print(f"           AwbGain(R/B)={actual_awb_gain_r}/{actual_awb_gain_b}")
                        print(f"           AeEnabled={ae_enabled},  AwbEnabled={awb_enabled}")

                    # SensorTimestampの蓄積とフレーム間Δ表示
                    sensor_timestamps.append(ts_sensor)
                    if len(sensor_timestamps) >= 2:
                        delta_sec = (sensor_timestamps[-1] - sensor_timestamps[-2]) / 1_000_000_000
                        print(f"   - [メタ] {exp_ok} ExposureTime={actual_exp_us}μs, AnalogueGain={actual_analog_gain}")
                        print(f"           SensorTimestamp={ts_sensor} ns [SensorΔ={delta_sec:.3f}s]")
                    else:
                        print(f"   - [メタ] {exp_ok} ExposureTime={actual_exp_us}μs, AnalogueGain={actual_analog_gain}")
                        print(f"           SensorTimestamp={ts_sensor} ns")
                    raw_buffer = request.make_array("raw")

                    if not isinstance(raw_buffer, np.ndarray) or len(raw_buffer.shape) != 2:
                        raise ValueError(f"RAWデータが期待外れの形式です: {type(raw_buffer)}, {raw_buffer.shape}")

                    # ── 生 RAW データの安全コピー (メモリ独立性を確保) ───
                    raw_copy = np.array(raw_buffer).copy()  # ← ※ dtype 制約なし (uint16 or uint8 保持)

                    # コピー後の統計情報を表示 (dtype により解釈が異なる)
                    raw_min  = int(np.min(raw_copy))
                    raw_max  = int(np.max(raw_copy))
                    raw_mean = float(np.mean(raw_copy))
                    print(f"   - [RAW] dtype={raw_copy.dtype}, min={raw_min}, max={raw_max}, mean={raw_mean:.1f} (shape={raw_copy.shape})")

                    # ── センサー固有パラメータを metadata から抽出 ──────────────
                    frame_black_level = _extract_black_level(metadata)
                    frame_sat_level   = _extract_saturation_level(metadata)
                    bayer_pattern     = BAUER_PATTERN_DETECTED  # 初期化時に1回だけ設定済み
                    
                    print(f"   - [センサー] BlackLevel={frame_black_level:.0f}, "
                          f"SaturationLevel={frame_sat_level:.0f}, "
                          f"BayerPattern={bayer_pattern}")

                    # RAWデータ + センサーパラメータの6要素タプルを作成し、キューに追加
                    picam_cap_queue.put((raw_copy, start_time_ms, frame_idx,
                                        frame_black_level, frame_sat_level, bayer_pattern))

                finally:
                    # リソースを確実に解放する（キャメラシステムへの回収）
                    request.release()

            except Exception as e: 
                if len(picam_cap_queue.queue) >= QUEUE_SIZE:
                    print(f"警告：キューが満杯になり、次のキャプチャをスキップします. [{e}]")
                else :
                    # キューサイズが十分ない場合はエラーを再発生させる
                    raise
            
            end_time_ms = time.monotonic() * 1000    

            cap_interval = round(end_time_ms - start_time_ms)

            print(f"   - 撮影にかかった時間: {cap_interval} ms | 現在のキューサイズ [{len(picam_cap_queue.queue)}/{QUEUE_SIZE}]")
            cap_intervals_ms.append(cap_interval)
                        
            # 1フレームごとの後処理完了待ちのイベントの設定
            # [修正#2] タイムOUT: cap_interval*1.2/1000 → ~0.5秒では露光4秒+後処理を待てない.
            #   露光時間(EXP_TIME_US) + 後処理余裕(1.5秒) に設定 (例: 4秒→~5.5秒)
            min_timeout = EXP_TIME_US / 1_000_000 + 1.5
            timeout_sec = max(min_timeout, cap_interval / 1000 * 1.2)
            if not post_proc_event.wait(timeout=timeout_sec):
                
                print(f"警告：次の撮影開始まで、後処理が終了しませんでした (タイムアウト: {timeout_sec:.1f} s).")
            
            # キャプチャと後処理が完了してから次フレームへ移行
            ts_frame_start = time.monotonic() * 1000

    except Exception as ex: 
        print(div_msg + f"<異常>: 撮影ループの途中で問題が発生しました.\n{ex}")
        
        # ポイズン・ピルを送信して後処理スレッドを終了させる
        if proc_thread.is_alive():
            picam_cap_queue.put(None)
            print("警告：後処理スレッドを終了させるためにポイズン・ピルを送信しました.")
        
        # 後処理スレッドが停止するまで待つ（タイムアウトは30秒）
        proc_thread.join(timeout=30)
        
        # カメラを確実に終了させる
        if not camera_stopped:
            picam2.stop()
            camera_stopped = True
        
        print(div_msg + "エラーにより撮影を中断しました. プログラムを終了します.")
        sys.exit(-1)

    # --- <キャプチャ・ループ> 完了後 --------------------------------------------------
    print(div_msg + "後処理・キャプチャの待機と終了中")
    
    # すべてのフレームの後処理が完了したら、キューキャンセル用の特殊なキュー要素 (None) を送信する
    if proc_thread.is_alive():
        print(f" - 後処理スレッド ({proc_thread.name}) の終了を通知するため、ポイズン・ピルを送信")
        picam_cap_queue.put(None)
    
    # 後処理スレッドが停止するまで待つ（タイムアウトは30秒）
    max_wait_sec  = 30

    # thread.join() は常に None を返すため、タイムアウト判定は is_alive() で行う
    proc_thread.join(timeout=max_wait_sec)
    
    
    if proc_thread.is_alive(): 
        print(f"警告：後処理スレッドが約 {max_wait_sec} 秒経っても停止しなかった。")
        
    # --- 統計情報を表示
    
    ts_end = time.monotonic() * 1000

    elapsed_sec = (ts_end - ts_start) / 1000
    
    print(div_msg + "測定結果" )

    print(f"- [測定時間]: {round(elapsed_sec)} s")
    print(f" - 取得されたコマ数: {len(proc_start_times)}.")
    
    if len(cap_intervals_ms) >= 2:
        gap_stats = {}
        gap_stats['最小']  = round(min(cap_intervals_ms))
        gap_stats['最大']  = round(max(cap_intervals_ms))
        try:
            gap_stats['平均']  = round(sum([g for g in cap_intervals_ms if g > 0]) / len(cap_intervals_ms)) 
        except ZeroDivisionError:
            gap_stats['平均'] = '--' 

        print(div_msg + "撮影間隔統計" )
        
        # タブ区切りで各項目を表示
        max_key_length = len(max(gap_stats.keys(), key=len)) + 4
        for k, v in gap_stats.items():
            stat_line = f" - {k:<{max_key_length}}: {v}"
            print(stat_line)

    else:
        print("警告：測定されたコマ数が2未満のため、統計表示はできません.")

    # --- SensorTimestamp間隔統計 ---
    if len(sensor_timestamps) >= 2:
        ts_diffs_sec = [(sensor_timestamps[i+1] - sensor_timestamps[i]) / 1_000_000_000
                        for i in range(len(sensor_timestamps)-1)]
        ts_stats = {}
        ts_stats["最小"]   = round(min(ts_diffs_sec), 3)
        ts_stats["最大"]   = round(max(ts_diffs_sec), 3)
        ts_stats["平均"]   = round(sum(ts_diffs_sec) / len(ts_diffs_sec), 3)

        print(div_msg + "SensorTimestamp間隔統計")

        max_key_length = len(max(ts_stats.keys(), key=len)) + 4
        for k, v in ts_stats.items():
            stat_line = f" - {k:<{max_key_length}}: {v} s"
            print(stat_line)

    else:
        print("警告：SensorTimestampが2未満のため、間隔統計表示はできません.")

    print(div_msg + "測定終了" )
    
    # ライブラリの停止
    picam2.stop()

# --- エントリポイント -----------------------------------------------------------
if __name__ == "__main__":
    main()
