# Picamera2Controller.py
# picamera2 ライブラリを用いた IMX477 カメラ制御クラス.
# CameraController と同じ属性・メソッドインタフェースを提供する.

import subprocess
import os
import threading
import time
import cv2
import numpy as np
import datetime
import queue
from picamera2 import Picamera2
# ==============================================================================
# モジュールレベル補助関数 (TestPicamera2.py から移植・簡素化)
# ==============================================================================

_DEFAULT_BLACK_LEVEL = 512.0    # IMX477/12bit BlackLevel
_DEFAULT_SAT_LEVEL = 4095.0     # IMX477/12bit 飽和レベル


def _detect_bayer_pattern(sensor_fmt: str, metadata: dict) -> str:
    """sensor_format名 + ColourFilterArray メタデータからBayerパターンを推測する."""
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

    sf = sensor_fmt.upper()
    if "BGGR" in sf:
        return "BGGR"
    elif "RGGB" in sf:
        return "RGGB"
    elif "GRBG" in sf:
        return "GRBG"
    elif "GBRG" in sf:
        return "GBRG"

    print(f"   WARN: Bayerパターン自動判定不可. センサー形式={sensor_fmt}. BGGRを仮定.")
    return "BGGR"


def _extract_black_level(metadata: dict, default_bl: float = _DEFAULT_BLACK_LEVEL) -> float:
    """metadata から BlackLevel を抽出する."""
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


def _extract_saturation_level(metadata: dict, default_wl: float = _DEFAULT_SAT_LEVEL) -> float:
    """metadata から SaturationLevel を抽出する."""
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
        print(f"   WARN: 有効なダイナミックレンジが{effective}以下. デフォルトスケールを使用します.")
        return 65535.0 / 4095.0
    return 65535.0 / effective


def _unpack_raw_2BPP(packed_uint8: np.ndarray, raw_h: int, raw_w: int, pq_alternating: bool = False) -> np.ndarray:
    """SBGGR12 / 2画素=3バイトパック形式を uint16 に展開。(フォールバック用)"""
    expected_bytes_per_row = (raw_w // 2) * 3
    result = np.empty((raw_h, raw_w), dtype=np.uint16)

    for r in range(raw_h):
        row_valid = packed_uint8[r, :expected_bytes_per_row].astype(np.uint16)
        n_triplets = len(row_valid) // 3
        triples = row_valid[:n_triplets * 3].reshape((n_triplets, 3))
        b0 = triples[:, 0]
        b1 = triples[:, 1]
        b2 = triples[:, 2]

        px0_p = b0 | ((b1 & 0xF0).astype(np.uint16) << 4)
        px1_p = (b2.astype(np.uint16) << 4) | (b1 & 0x0F)

        row_pixels = np.empty(n_triplets * 2, dtype=np.uint16)
        row_pixels[0::2] = px0_p
        row_pixels[1::2] = px1_p

        result[r, :] = row_pixels

    if np.max(result) > 4095:
        print(f"   WARN: Unpacked RAW max={np.max(result)} > 4095.")

    return result


# ==============================================================================
# Picamera2Controller クラス
# ==============================================================================

class Picamera2Controller:
    def __init__(self, stepping_motor_controller, globals):
        self.stepping_motor_controller = stepping_motor_controller
        self.globals = globals

        # --- CameraController と同一の属性 ---
        self.ExpSec = 0.25
        self.ExpMicSec = int(self.ExpSec * 1000000)
        self.fRunCamera = threading.Event()
        self.fRunCamera.clear()
        self.iCapture = 2
        self.iCapRead = 2
        self.fImageReady = threading.Event()
        self.fImageReady.clear()
        self.AnalogGain = 16
        self.WBGR = 1.4
        self.WBGG = 1.2
        self.WBGB = 1.5
        self.WBGL = 1.0
        self.SensorW = 4056
        self.SensorH = 3040
        self.xZoomCenter = int(self.SensorW / 2)
        self.yZoomCenter = int(self.SensorH / 2)
        self.ZoomWindowHW = 128
        self.FrameImage = np.zeros((self.SensorH, self.SensorW, 3), dtype=np.uint16)
        self.StackImage = np.array(self.FrameImage * 0, dtype=np.float32)
        self.CropImage = self.FrameImage[self.yZoomCenter - self.ZoomWindowHW:self.yZoomCenter + self.ZoomWindowHW,
                                         self.xZoomCenter - self.ZoomWindowHW:self.xZoomCenter + self.ZoomWindowHW]
        self.ZoomImage = (self.CropImage / 16).clip(2, 255).astype(np.uint8)
        self.DarkImage = np.zeros((self.SensorH, self.SensorW, 3), dtype=np.uint16)
        self.vThreshold = 128
        self.vZAtten = 16.0     # CameraController/dcraw版と同等の初期値 (RPi4互換性)
        self.fDark = False
        self.fTrack = 0
        self.fBaseSet = 0
        self.BaseX = self.xZoomCenter
        self.BaseY = self.yZoomCenter
        self.ShiftX = 0
        self.ShiftY = 0
        self.fLost = 0
        self.fStackBusy = 0
        self.StackCounter = 0
        self.MStack = 64
        self.MaxStackReached = 0
        self.fRunImageUpdate = threading.Event()
        self.fRunImageUpdate.clear()
        self.run_thread = None
        self.convert_thread = None
        self.stop_run_event = threading.Event()
        self.stop_convert_event = threading.Event()

        # --- デバッグ用: 全フレームの画像保存機能 ---
        self.debug_save_frames = False      # False でPNG保存オフ（16bit RAW→Tkinter表示確認用）
        self.debug_frame_dir = "debug_frames"

        # --- 露出/ゲイン変更のリアルタイム反映用追跡変数 ---
        self._last_exposure = self.ExpMicSec
        self._last_analoggain = self.AnalogGain

        # --- picamera2 固有のメンバ ---
        self.picam2 = None
        self.raw_queue = queue.Queue(maxsize=2)  # ダブルバッファ用キュー
        self.bayer_pattern = "BGGR"    # デフォルト: IMX477 は BGGR
        self.black_level = 512.0       # IMX477/12bit のBlackLevel (デフォルト値)
        self.saturation_level = 4095.0  # IMX477/12bit のADC飽和レベル (2^12-1)

    def reset_camera(self):
        """カメラ初期化・プレキャプチャ (picamera2版).
        
        CameraController.reset_camera() と同等の動作を picamera2 で再現する.
        DPC設定 → Picamera2インスタンス作成 → STILL+RAWconfigure → カメラ起動
        → プレキャプチャでBayer/BlackLevel/Saturation取得 → FrameImage初期化.
        """
        # --- ① DPC設定 (既存のCameraControllerと同等) ---
        StrCommand = "sudo vcdbg set imx477_dpc 3"
        try:
            res = subprocess.check_call(StrCommand, shell=True)
            print("DPC is Set")
        except subprocess.CalledProcessError as e:
            print(f"DPC is NOT Set: {e}")

        # --- ② Picamera2 インスタンス作成 + 構成 ---
        self.picam2 = Picamera2()

        # STILL (LOWRES) 用メイン構成
        # ★ RAWストリームは SBGGR12 を明示指定 (libcamera auto選択の環境依存を防止)
        main_config = self.picam2.create_still_configuration(
            main={"size": (self.SensorW, self.SensorH), "format": "RGB888"},
            raw={"format": "SBGGR12", "size": (self.SensorW, self.SensorH)},
        )
        self.picam2.configure(main_config)

        # --- ③ AE/AWB無効化 + 手動露出設定を start() 前に実施 ---
        # ★ set_controls() は「次のリクエスト」に適用されるため、start() より前にセット必須.
        #   (TestPicamera2.py と同様の順序)
        self.picam2.set_controls({
            "AeEnable": False,
            "AwbEnable": False,
            "ExposureTime": self.ExpMicSec,
            "AnalogueGain": self.AnalogGain,
            "ColourGains": (self.WBGR, self.WBGB),  # (R_gain, B_gain)
        })

        # --- ④ カメラ起動 ---
        self.picam2.start()

        # --- ⑤ プレキャプチャ (Bayerパターン + BlackLevel + Saturation取得) ---
        dt_now = datetime.datetime.now()
        StrCapture = "Pre Capture 1 at " + dt_now.strftime('%Y-%m-%d %H:%M:%S')
        print(StrCapture)

        # 最初のプレキャプチャでメタデータを取得
        self.fImageReady.clear()
        request1 = self.picam2.capture_request()
        metadata1 = request1.get_metadata()

        # sensor_format の取得（デバッグ用）
        sensor_fmt = getattr(self.picam2.camera, "_sensor_format", None) or "unknown"

        # Bayerパターン自動検出
        self.bayer_pattern = _detect_bayer_pattern(sensor_fmt, metadata1)
        print(f"Bayer pattern: {self.bayer_pattern}")

        # BlackLevel / SaturationLevel 取得（metadataにキーがない場合はデフォルト値を保つ）
        self.black_level = _extract_black_level(metadata1, self.black_level)
        self.saturation_level = _extract_saturation_level(metadata1, self.saturation_level)
        
        # フォールバック: 値が0のままだとscale=infになるため、明示的にデフォルトを上書き
        if self.black_level <= 0:
            self.black_level = float(_DEFAULT_BLACK_LEVEL)
            print(f"   WARN: BlackLevel was 0, using default {self.black_level}")
        if self.saturation_level <= 0:
            self.saturation_level = float(_DEFAULT_SAT_LEVEL)
            print(f"   WARN: SaturationLevel was 0, using default {self.saturation_level}")
        
        # BlackLevel < SaturationLevel であることを保証（否则scaleが負値や0になる）
        if self.black_level >= self.saturation_level:
            self.black_level = float(_DEFAULT_BLACK_LEVEL)
            self.saturation_level = float(_DEFAULT_SAT_LEVEL)
            print(f"   WARN: BL>=SAT, reverting to defaults BL={self.black_level}, SAT={self.saturation_level}")
        
        print(f"BlackLevel: {self.black_level:.1f}, SaturationLevel: {self.saturation_level:.1f}")

        request1.release()

        # プレキャプチャ2 (CameraControllerと同等のタイミング)
        dt_now = datetime.datetime.now()
        StrCapture = "Pre Capture 2 at " + dt_now.strftime('%Y-%m-%d %H:%M:%S')
        print(StrCapture)

        self.fImageReady.clear()
        request2 = self.picam2.capture_request()
        metadata2 = request2.get_metadata()
        request2.release()

        # --- ⑤ FrameImage初期化 + fImageReadyセット ---
        self.FrameImage = np.zeros((self.SensorH, self.SensorW, 3), dtype=np.uint16)
        self.fImageReady.set()

        dt_now = datetime.datetime.now()
        print(f"reset_camera() completed at {dt_now.strftime('%Y-%m-%d %H:%M:%S')}")

    # ==========================================================================
    # 露出/ゲインのリアルタイム更新メソッド (CameraController と同等のインタフェース)
    # ==========================================================================

    def set_exposure(self, exposure_us):
        """ExposureTime をリアルタイムで設定.

        CameraController では ``-ss`` フラグに ExpMicSec を渡していたのと同等の役割.
        GUI ⇨ GUIHandler.change_expos() ⇨ self.ExpMicSec 更新 ⇨ このメソッドを直接呼ぶ、
        または run_camera ループ内のポーリングが自動的に呼び出す.

        Args:
            exposure_us (int): マイクロ秒単位の露出時間.
        """
        self.ExpMicSec = int(exposure_us)
        self.ExpSec = self.ExpMicSec / 1000000.0
        self._last_exposure = self.ExpMicSec
        print(f"set_exposure called: {self.ExpMicSec} µs ({self.ExpSec:.3f}s)")
        if self.picam2 is not None and self.fRunCamera.is_set():
            try:
                self.picam2.set_controls({"ExposureTime": self.ExpMicSec})
            except Exception as e:
                print(f"Warning: Failed to set exposure time: {e}")

    def set_analoggain(self, gain):
        """AnalogueGain をリアルタイムで設定.

        CameraController では ``-ag`` フラグに AnalogGain を渡していたのと同等の役割.
        ※ set_controls() は行わず、run_camera() ループでの差分検出に任せる.

        Args:
            gain: アナログゲイン値.
        """
        self.AnalogGain = float(gain)
        print(f"set_analoggain called: {self.AnalogGain}")

    # ==========================================================================
    # キャプチャ・後処理スレッド本体
    # ==========================================================================

    def run_camera(self):
        """キャプチャスレッド本体.

        CameraController.run_camera() と同等の動作を picamera2 で再現する.
        - fRunCamera イベントがセットされている間、RAWフレームをキャプチャ.
        - self.raw_queue に (raw_array, metadata) タプルを投入.
        - stop_run_event がセットされるとループを脱出.
        """
        dt_now = datetime.datetime.now()
        print(f"run_camera thread started at {dt_now.strftime('%Y-%m-%d %H:%M:%S')}")

        while not self.stop_run_event.is_set():
            # fRunCamera がセットされていない場合は待機
            if not self.fRunCamera.is_set():
                time.sleep(0.1)
                continue

            # fRunCamera がセットされたらその旨をログ (初回のみ)
            if not getattr(self, '_capture_started', False):
                self._capture_started = True

            # --- GUI からの露出/ゲイン変更をリアルタイムに反映 ---
            needs_update = False
            new_controls = {}

            if self.ExpMicSec != self._last_exposure:
                new_controls["ExposureTime"] = self.ExpMicSec
                self._last_exposure = self.ExpMicSec
                needs_update = True

            if self.AnalogGain != self._last_analoggain:
                new_controls["AnalogueGain"] = self.AnalogGain
                self._last_analoggain = self.AnalogGain
                needs_update = True

            if needs_update and self.picam2 is not None:
                try:
                    self.picam2.set_controls(new_controls)
                except Exception as ctrl_err:
                    print(f"Warning: Failed to set camera controls: {ctrl_err}")

            try:
                dt_now = datetime.datetime.now()
                StrCapture = "CAPTURE " + dt_now.strftime('%Y-%m-%d %H:%M:%S')

                self.fImageReady.clear()

                request = self.picam2.capture_request()

                try:
                    # 生RAWデータを安全にコピー (メモリ独立性を確保)
                    raw_array = request.make_array("raw").copy()
                    metadata = request.get_metadata().copy()

                    # キューに投入（満杯時は古いものを優先的に破棄）
                    if self.raw_queue.full():
                        try:
                            self.raw_queue.get_nowait()  # 未処理の古いフレームを破棄
                        except queue.Empty:
                            pass

                    self.raw_queue.put((raw_array, metadata))
                finally:
                    request.release()

            except Exception as e:
                print(f"run_camera error: {e}")
                time.sleep(0.5)

        dt_now = datetime.datetime.now()
        print(f"run_camera thread stopped at {dt_now.strftime('%Y-%m-%d %H:%M:%S')}")

    def convert_raw(self):
        """RAW後処理スレッド本体.

        CameraController.convert_raw() と同等の動作を picamera2 で再現する.
        - self.raw_queue から (raw_array, metadata) を取得.
        - RAWデータ形式判定 → uint16再構築(12bit unpack).
          * Pi4: 2BPP packed (画素2=3バイト) → `_unpack_raw_2BPP()` で展開.
          * Pi5/PISP: BYR2 (uint8×2/画素+padding) → `view(np.uint16)` で再解釈.
        - BlackLevel引算 → WB補正 → 12→16bitスケール → デモザイキング.
        - self.FrameImage に格納して fImageReady.set().
        - stop_convert_event がセットされるとループを脱出.
        """
        dt_now = datetime.datetime.now()
        print(f"convert_raw thread started at {dt_now.strftime('%Y-%m-%d %H:%M:%S')}")

        while not self.stop_convert_event.is_set():
            try:
                if not self.fRunCamera.is_set():
                    time.sleep(0.1)
                    continue

                # 指定時間以内に取り出せなければスキップ
                try:
                    item = self.raw_queue.get(timeout=0.5)
                except queue.Empty:
                    continue

                if item is None:
                    continue

                raw_data, metadata = item

            except queue.Empty:
                continue
            except Exception as e:
                import traceback
                print(f">>> [CONVERT] Queue error: {e}")
                traceback.print_exc()
                time.sleep(0.1)
                continue

            try:
                # --- ① RAW配列のdtype判定・unpack処理 (12bit→uint16再構築) ---
                # Pi4/libcamera v0.4: uint8 (2BPP packed SBGGR12, 画素2=3バイト)
                #   bytes_per_row ≈ (SensorW//2)*3 = 6084
                # Pi5/PISP:           uint8 × SensorW*2 + padding (BYR2 little-endian, 画素毎に2バイト).
                #   shape例: (3040, 8128) → SensorW*2=8112 + pad 16 バイト. → uint16 view で再構築.
                if raw_data.dtype == np.uint8:
                    raw_h, raw_w_bytes = raw_data.shape[:2]
                    expected_2bpp_bpr = (self.SensorW // 2) * 3   # 2BPP の場合の期待バイト幅 (=6084)
                    threshold_2bpp = expected_2bpp_bpr * 0.1       # ±10% 許容 (+-608 バイト)

                    if abs(raw_w_bytes - expected_2bpp_bpr) < threshold_2bpp:
                        # ─── パターンA: 2BPP packed (Pi4/libcamera v0.4, 画素2=3バイト) ───
                        rd16 = _unpack_raw_2BPP(raw_data, raw_h, self.SensorW)

                    elif abs(raw_w_bytes - self.SensorW * 2) < 64:
                        # ─── パターンB: PISP BYR2 (Pi5, uint8×2/画素 + 最大64バイトまでのpadding) ───
                        # 各行の先頭 SensorW*2 バイトを取り出し, little-endian で uint16 に再解釈.
                        sliced = raw_data[:, :self.SensorW * 2].copy()
                        rd16 = sliced.view(np.uint16).reshape(raw_h, self.SensorW)

                    else:
                        # ─── パターンC: 未知の形式 (フォールバック) ───
                        rd16 = raw_data.astype(np.uint16)[:, :self.SensorW]

                elif raw_data.dtype == np.uint16:
                    # ─── libcamera v0.5 / PISP で uint16 が直接返される場合 ───
                    rd16 = raw_data[:, :self.SensorW].copy()
                else:
                    rd16 = raw_data.astype(np.uint16)[:, :self.SensorW].copy()

                # --- ② BlackLevel引算 → WB補正 → 16bitスケール (float32で演算) ---
                # rd16 が正常に uint16 で読み出せていれば、range は 0〜4095 (12bit).

                rd_min = int(rd16.min())
                rd_max = int(rd16.max())

                frame_bl = self.black_level
                if rd_max <= 255:
                    # 8bitスケール(PISP後処理済みと判断). 8→16bit stretch.
                    scale = 65535.0 / 255.0
                    frame_bl = 0.0  # 8bitデータではBlackLevel引き算を行わない（既に処理済みのため）

                elif rd_max <= 4095:
                    # standard 12bit raw (IMX477/Pi4 legacy). full stretch.
                    scale = 65535.0 / 4095.0

                else:
                    # uint16 のまま既に16bit. scale=1.0
                    scale = 1.0


                rb = rd16.astype(np.float32)
                # ★ BlackLevel引算（float32安全: np.maximum で負の値を防止）
                if frame_bl > 0:
                    rb -= frame_bl
                    rb = np.maximum(rb, 0.0)

                # WBゲイン適用（Bayerパターンに応じた位置にR/G/Bを掛ける）
                # ※ 8bit PISP後処理済みデータでは既にWBが適用済みなのでスキップ
                if rd_max > 255:
                    wb_r = self.WBGR
                    wb_g = self.WBGG
                    wb_b = self.WBGB

                    if self.bayer_pattern == "BGGR":
                        rb[0::2, 0::2] *= wb_b  # B
                        rb[0::2, 1::2] *= wb_g  # G
                        rb[1::2, 0::2] *= wb_g  # G
                        rb[1::2, 1::2] *= wb_r  # R
                    elif self.bayer_pattern == "RGGB":
                        rb[0::2, 0::2] *= wb_r  # R
                        rb[0::2, 1::2] *= wb_g  # G
                        rb[1::2, 0::2] *= wb_g  # G
                        rb[1::2, 1::2] *= wb_b  # B
                    elif self.bayer_pattern == "GRBG":
                        rb[0::2, 0::2] *= wb_g
                        rb[0::2, 1::2] *= wb_r
                        rb[1::2, 0::2] *= wb_b
                        rb[1::2, 1::2] *= wb_g
                    elif self.bayer_pattern == "GBRG":
                        rb[0::2, 0::2] *= wb_g
                        rb[0::2, 1::2] *= wb_b
                        rb[1::2, 0::2] *= wb_r
                        rb[1::2, 1::2] *= wb_g
                else:
                    pass  # 8bit PISP後処理済みの場合はWBゲインスキップ

                # スケール適用 + clip
                rb *= scale
                rc = np.clip(rb, 0, 65535).astype(np.uint16)

                # --- ③ デモザイキング (cv2.cvtColor: Bayer→RGB, uint16対応) ---
                if self.bayer_pattern == "BGGR":
                    img_rgb = cv2.cvtColor(rc, cv2.COLOR_BayerBG2RGB)
                elif self.bayer_pattern == "RGGB":
                    img_rgb = cv2.cvtColor(rc, cv2.COLOR_BayerRG2RGB)
                elif self.bayer_pattern == "GRBG":
                    img_rgb = cv2.cvtColor(rc, cv2.COLOR_BayerGR2RGB)
                elif self.bayer_pattern == "GBRG":
                    img_rgb = cv2.cvtColor(rc, cv2.COLOR_BayerGB2RGB)
                else:
                    print(f"   WARN: 未知のBayerパターン '{self.bayer_pattern}', BGGRフォールバック")
                    img_rgb = cv2.cvtColor(rc, cv2.COLOR_BayerBG2RGB)

                # ※ RGB→BGR スワップ (Pi5メモリ直接格納版).
                # CameraController(dcraw+cv2.imread tiff) はBGRとして出力するため、
                # こちらも同じ色順序にしてGUIの挙動をそろえる.
                img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)

                # --- ④ FrameImage にコピー + fImageReady セット ---
                self.FrameImage[:] = img_bgr.copy()
                self.fImageReady.set()

                # --- デバッグ用: フレーム画像をタイムスタンプ付きファイルに保存 ---
                if self.debug_save_frames:
                    dt_save = datetime.datetime.now()
                    fname = f"frame_{dt_save.strftime('%Y%m%d_%H%M%S')}_{dt_save.microsecond}.png"
                    os.makedirs(self.debug_frame_dir, exist_ok=True)
                    save_path = os.path.join(self.debug_frame_dir, fname)
                    lo = float(np.percentile(img_rgb, 1))
                    hi = float(np.percentile(img_rgb, 99))
                    if hi - lo < 1:
                        lo, hi = 0.0, 65535.0
                    rng_ = hi - lo
                    img_u8 = np.clip((img_rgb.astype(np.float32) - lo) / rng_ * 255.0, 0, 255).astype(np.uint8)
                    img_bgr_debug = img_u8[..., ::-1]
                    cv2.imwrite(save_path, img_bgr_debug)


            except Exception as e:
                import traceback
                print(f">>> [CONVERT] Processing error: {e}")
                traceback.print_exc()
                self.fImageReady.clear()
                time.sleep(0.1)

        dt_now = datetime.datetime.now()
        print(f"convert_raw thread stopped at {dt_now.strftime('%Y-%m-%d %H:%M:%S')}")

    # ==============================================================================
    # CaptureFrames / DetectShift (CameraController と同等のインタフェース)
    # ==============================================================================

    def CaptureFrames(self):
        """フレーム取得・ステッキング処理。

        CameraController の CaptureFrames と同等ですが、
        temp/capture.tiff のファイルI/O ではなく self.FrameImage を直接使用します。
        """
        if not self.fImageReady.is_set():
            time.sleep(0.1)
            return

        # FrameImage を直接コピー（元の cv2.imread('temp/capture.tiff') に相当）
        ReadImage = self.FrameImage.copy()
        self.fImageReady.clear()

        if (self.fTrack == 1):
            self.globals.fDetShift = 0
            self.DetectShift()
            if (self.globals.fDetShift == 2) and (self.StackCounter < self.MStack):
                if self.fLost == 1:
                    self.StackCounter = 0
                    self.MaxStackReached = 0
                    self.StackImage *= 0
                    self.BaseX = self.xZoomCenter
                    self.BaseY = self.yZoomCenter
                    self.globals.fDetShift = 0
                    print("STACKING LOST")
                else:
                    if int(self.StackCounter) == 0:
                        self.fBaseSet = 0

        # --- ステッキング処理 ---
        if not self.fStackBusy and (self.StackCounter < self.MStack):
            self.fStackBusy = 1
            dt_now = datetime.datetime.now()
            StrCapture = "STACKING " + dt_now.strftime('%Y-%m-%d %H:%M:%S')
            print(StrCapture)

            # --- StackImage に加算（DarkImageを引いた後）---
            self.StackImage += (ReadImage.astype(np.float32) - self.DarkImage.astype(np.float32))
            self.StackCounter += 1

            if self.StackCounter >= self.MStack:
                self.MaxStackReached = 1
                self.fRunCamera.clear()
                self.fStackBusy = 0
                self.globals.fRunDisplayUpdate = 1
                self.globals.fRunZoomUpdate = 1
                dt_now = datetime.datetime.now()
                StrCapture = "STACKING " + dt_now.strftime('%Y-%m-%d %H:%M:%S') + " Stacked"
                print(StrCapture)
            else:
                # まだ完全なスタックには達していない
                self.fStackBusy = 0
                self.globals.fRunDisplayUpdate = 1
                self.globals.fRunZoomUpdate = 1
                dt_now = datetime.datetime.now()
                StrCapture = "BUFFER " + dt_now.strftime('%Y-%m-%d %H:%M:%S') + " Stacked"
                print(StrCapture)

        time.sleep(0.1)

    def DetectShift(self):
        """星追尾用シフト検出。

        CameraController の DetectShift と同等です。
        """
        self.CropImage = self.FrameImage[self.yZoomCenter - self.ZoomWindowHW:self.yZoomCenter + self.ZoomWindowHW,
                                        self.xZoomCenter - self.ZoomWindowHW:self.xZoomCenter + self.ZoomWindowHW]
        # ★ uint16→uint8: /vZAtten(/16程度)で正規化しGUIスライダーと互換性を維持.
        # CameraController/dcraw版と同様に int(vZAtten) を直接使用する（RPi4互換性）.
        self.ZoomImage = (self.CropImage / int(self.vZAtten)).clip(2, 255).astype(np.uint8)
        GrayFrame = cv2.cvtColor(self.ZoomImage, cv2.COLOR_RGB2GRAY)
        BlurFrame = cv2.blur(GrayFrame, (5, 5))
        ret, GrayFrame0 = cv2.threshold(BlurFrame, int(self.vThreshold), 255, cv2.THRESH_BINARY)
        contours, hierarchy = cv2.findContours(GrayFrame0, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if (len(contours) == 1):
            cnt = contours[0]
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            if(radius < 32):
                if(self.fBaseSet == 0):
                    self.BaseX = int(x) + self.xZoomCenter - self.ZoomWindowHW
                    self.BaseY = int(y) + self.yZoomCenter - self.ZoomWindowHW
                    # Recentering
                    self.xZoomCenter += (int(x) - self.ZoomWindowHW)
                    self.yZoomCenter += (int(y) - self.ZoomWindowHW)
                    self.fBaseSet = 1
                else:
                    self.ShiftX = int(x) + self.xZoomCenter - self.ZoomWindowHW - self.BaseX
                    self.ShiftY = int(y) + self.yZoomCenter - self.ZoomWindowHW - self.BaseY
                    self.xZoomCenter += int(x) - self.ZoomWindowHW
                    self.yZoomCenter += int(y) - self.ZoomWindowHW
                    if(self.ShiftX > 5):  # Positive = Star moves faster so speeding up
                        self.stepping_motor_controller.TimeStep1 = self.stepping_motor_controller.GuideStep * 0.8
                    elif(self.ShiftX < -5):  # Negative = star moves slowly so speeding down
                        self.stepping_motor_controller.TimeStep1 = self.stepping_motor_controller.GuideStep * 1.2
                    if(self.ShiftY > 5):
                        self.stepping_motor_controller.SetLoops2(-0.0026)
                    elif(self.ShiftY < -5):
                        self.stepping_motor_controller.SetLoops2(0.0026)
                    self.fLost = 0
            else:
                self.fLost = 1
        else:
            self.fLost = 1

    # ==============================================================================
    # スレッド制御メソッド (CameraController と同等のインタフェース)
    # ==============================================================================

    def start_run_thread(self):
        """キャプチャスレッドを起動."""
        self.stop_run_event.clear()
        self.run_thread = threading.Thread(target=self.run_camera, daemon=True)
        self.run_thread.start()
        print("run_camera thread started")

    def stop_run_thread(self):
        """キャプチャスレッドを停止."""
        if self.run_thread:
            self.stop_run_event.set()
            self.run_thread.join(timeout=5)
            if self.run_thread.is_alive():
                print("run_thread is still alive, forcing termination")

    def start_convert_thread(self):
        """RAW後処理スレッドを起動."""
        self.stop_convert_event.clear()
        self.convert_thread = threading.Thread(target=self.convert_raw, daemon=True)
        self.convert_thread.start()
        print("convert_raw thread started")

    def stop_convert_thread(self):
        """RAW後処理スレッドを停止."""
        if self.convert_thread:
            self.stop_convert_event.set()
            self.convert_thread.join(timeout=5)
            if self.convert_thread.is_alive():
                print("convert_thread is still alive, forcing termination")

