# CameraController.py
# ==============================================================================
# Raspberry Pi 4 / Pi 5 両対応のカメラ制御クラス.
#
# カメラバックエンド自動検知:
#   - 1st:  Picamera2Controller (picamera2 ライブラリベース)
#           Pi5(PISP搭載) または picamera2 がインストールされた環境で動作.
#   - 2nd:  CameraControllerRaspistill (raspistill + dcraw=subprocess ベース)
#           Pi4/Bookworm以前で raspistill コマンドが利用可能であればこれをフォールバックとして使用する.
#
# インターフェースは両実装で完全一致しているため、MainApp.py のインポート文を
# 変更することなく自動的に新しいモジュールを使用できる.
# ==============================================================================

import subprocess
import threading
import os


# ---------------------------------------------------------------------------
# カメラバックエンド自動検知 (ラズパイバージョン・環境に応じた切り替え)
# ---------------------------------------------------------------------------


def _detect_chip() -> str:
    """SoC チップのモデル文字列を返す."""
    try:
        with open('/proc/cpuinfo', 'r') as f:
            for line in f:
                if line.startswith('Hardware'):
                    return line.split(':')[1].strip().upper()
    except Exception:
        pass
    return ''


def _is_raspberry_pi5() -> bool:
    """BCM2712 (Raspberry Pi 5) かどうかを判定する."""
    chip = _detect_chip()
    return 'BCM2712' in chip or 'BCM283XX' not in chip and 'BCM2712' in chip


def _has_raspistill() -> bool:
    """raspistill コマンドが利用可能なenvかを判定する."""
    ret = subprocess.run(['which', 'raspistill'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return ret.returncode == 0


def _has_picamera2() -> bool:
    """picamera2 パッケージの import が成功するかを判定する."""
    try:
        from picamera2 import Picamera2
        return True
    except (ImportError, Exception):
        return False


# ---------------------------------------------------------------------------
# カメラバックエンド選択ファサード.
#
# 動作原則:
#   1. Raspberry Pi 5 (BCM2712) の場合 → 必須 picamera2 (raspistill は存在しない)
#   2. picamera2 がインストールされており raspistill が無い場合 → picamera2
#   3. それ以外 (Pi4 + raspistill あり など) → raspistill ベース
# ---------------------------------------------------------------------------


def _auto_select_camera_backend():
    """環境に応じて最適なカメラバックエンドクラスを返す."""

    pi5 = _is_raspberry_pi5()
    has_picam2 = _has_picamera2()
    has_rasp = _has_raspistill()

    # Pi5 は raspistill が存在しないため picamera2 に強制. picamera2 がない場合はエラー.
    if pi5:
        if has_picam2:
            print(">>> [CameraController] Raspberry Pi 5 検出 → Picamera2Controller を使用します")
            from Picamera2Controller import Picamera2Controller
            return Picamera2Controller
        else:
            raise ImportError(
                "Raspberry Pi 5 (BCM2712) を検出しましたが、picamera2 がインストールされていません.\n"
                "「pip install picamera2」を実行してください."
            )

    # Pi4 系などで両方のパスが利用可能な場合の優先順位判定:
    # - picamera2 ある + raspistillがない → picamera2 と強制
    if has_picam2 and not has_rasp:
        print(">>> [CameraController] raspistill が利用不可 (Bookwormなど) → Picamera2Controller を使用します")
        from Picamera2Controller import Picamera2Controller
        return Picamera2Controller

    # picamera2 も raspistill もある場合: raspistill ベースをデフォルト優先 (従来の挙動維持)
    # 環境変数 PICAMERA_BACKEND=picam2 で picamera2 を強制できる.
    force_backend = os.environ.get('PICAMERA_BACKEND', '').upper()

    if force_backend == 'PICAM2' and has_picam2:
        print(">>> [CameraController] 環境変数 PICAMERA_BACKEND=picam2 → Picamera2Controller を強制使用します")
        from Picamera2Controller import Picamera2Controller
        return Picamera2Controller

    # デフォルト: raspistill ベース (既存のコード)
    print(">>> [CameraController] CameraControllerRaspistill (raspistill/dcraw subprocess ベース) を使用します")
    # ↓ below で定義する raspistill ベースのクラス. import 回避のためここで関数を return
    return None


# ==============================================================================
# Raspistill + dcraw (subprocess) ベースのカメラ制御クラス
# ==============================================================================
import cv2
import numpy as np
import datetime
import time


class CameraControllerRaspistill:
    def __init__(self, stepping_motor_controller, globals):
        self.stepping_motor_controller = stepping_motor_controller
        self.globals = globals

        self.Raspistill = 'raspistill -md 3 -ex off -awb off -awbg 1.6,1.7 -drc off -st -t 60 -bm -r -n -q 70 '
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
        self.FrameImage = np.zeros((self.SensorH, self.SensorW,3), dtype=np.uint16)
        self.StackImage = np.array(self.FrameImage * 0, dtype=np.float32)
        self.CropImage = self.FrameImage[self.yZoomCenter - self.ZoomWindowHW:self.yZoomCenter + self.ZoomWindowHW,
                                        self.xZoomCenter - self.ZoomWindowHW:self.xZoomCenter + self.ZoomWindowHW]
        self.ZoomImage = (self.CropImage / 16).clip(2, 255).astype(np.uint8)
        self.DarkImage = np.zeros((self.SensorH, self.SensorW,3), dtype=np.uint16)
        self.vThreshold = 128
        self.vZAtten = 16.0
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

    def reset_camera(self):
        StrCommand = "sudo vcdbg set imx477_dpc 3"
        try:
            res = subprocess.check_call(StrCommand, shell=True)
            print("DPC is Set")
        except subprocess.CalledProcessError as e:
            print(f"DPC is NOT Set: {e}")
        StrCommand = self.Raspistill + ' -o temp/capture1.jpg -ss ' + str(self.ExpMicSec) + ' -ag ' + str(self.AnalogGain)
        dt_now = datetime.datetime.now()
        StrCapture = "Pre Capture 1 at " + dt_now.strftime('%Y-%m-%d %H:%M:%S')
        print(StrCapture)
        try:
            res = subprocess.check_call(StrCommand, shell=True)
        except subprocess.CalledProcessError as e:
            print(f"Capture Error: {e}")
        StrCommand = self.Raspistill + ' -o temp/capture2.jpg -ss ' + str(self.ExpMicSec) + ' -ag ' + str(self.AnalogGain)
        dt_now = datetime.datetime.now()
        StrCapture = "Pre Capture 2 at " + dt_now.strftime('%Y-%m-%d %H:%M:%S')
        print(StrCapture)
        try:
            res = subprocess.check_call(StrCommand, shell=True)
        except subprocess.CalledProcessError as e:
            print(f"Capture Error: {e}")
        
        StrCommand = 'mv temp/capture1.jpg temp/capture.jpg'
        try:
            res = subprocess.check_call(StrCommand, shell=True)
        except subprocess.CalledProcessError as e:
            print(f"Rename capture1.jpg to capture.jpg Error: {e}")
        
        WBGOption = '-r %4.3f %4.3f %4.3f %4.3f' % (self.WBGR, self.WBGG, self.WBGB, self.WBGL)
        StrCommand = 'dcraw -T -4 -q 0 temp/capture.jpg'
        print(StrCommand)
        res = subprocess.check_call(StrCommand, shell=True)
        dt_now = datetime.datetime.now()
        StrCapture = "Tif pre-conversion at " + dt_now.strftime('%Y-%m-%d %H:%M:%S')
        print(StrCapture)
        self.fImageReady.set()
        print("fImageReady value:", self.fImageReady.is_set())
        self.FrameImage = cv2.imread('temp/capture.tiff', -1)
        self.StackImage = np.array(self.FrameImage * 0, dtype=np.float32)
        self.CropImage = self.FrameImage[self.yZoomCenter - self.ZoomWindowHW:self.yZoomCenter + self.ZoomWindowHW,
                                        self.xZoomCenter - self.ZoomWindowHW:self.xZoomCenter + self.ZoomWindowHW]
        self.ZoomImage = (self.CropImage / 16).clip(2, 255).astype(np.uint8)

    def run_camera(self):
        while not self.stop_run_event.is_set():
            if not self.fRunCamera.is_set():
                time.sleep(0.1)
                continue
            
            if (self.iCapture == self.iCapRead):
                if self.iCapture == 1:
                    StrCommand = self.Raspistill + ' -o temp/capture2.jpg -ss ' + str(self.ExpMicSec) + ' -ag ' + str(self.AnalogGain)
                elif self.iCapture == 2:
                    StrCommand = self.Raspistill + ' -o temp/capture1.jpg -ss ' + str(self.ExpMicSec) + ' -ag ' + str(self.AnalogGain)
                try:
                    subprocess.check_call(StrCommand, shell=True)
                except subprocess.CalledProcessError as e:
                    print(f"Error running camera: {e}")
                    continue
                
                dt_now = datetime.datetime.now()
                StrCapture = "CAMERA " + dt_now.strftime('%Y-%m-%d %H:%M:%S') + " Capture%d" % self.iCapture
                print(StrCapture)
                
                self.iCapture += 1
                if (self.iCapture > 2):
                    self.iCapture = 1
                
                time.sleep(0.1)

    def convert_raw(self):
        while not self.stop_convert_event.is_set():
            while self.fRunCamera.is_set():
                if (self.iCapture == self.iCapRead):
                    time.sleep(0.1)
                    continue
                elif not self.fImageReady.is_set():
                    if self.iCapRead == 1:
                        StrCommand = 'mv temp/capture1.jpg temp/capture.jpg'
                        try:
                            subprocess.check_call(StrCommand, shell=True)
                        except subprocess.CalledProcessError as e:
                            print(f"Rename capture1.jpg to capture.jpg Error: {e}")
                    elif self.iCapRead == 2:
                        StrCommand = 'mv temp/capture2.jpg temp/capture.jpg'
                        try:
                            subprocess.check_call(StrCommand, shell=True)
                        except subprocess.CalledProcessError as e:
                            print(f"Rename capture2.jpg to capture.jpg Error: {e}")
                    StrCommand = 'dcraw -T -4 -q 0 temp/capture.jpg'
                    try:
                        subprocess.check_call(StrCommand, shell=True)
                    except subprocess.CalledProcessError as e:
                        print(f"Error converting raw image: {e}")
                        continue
                    dt_now = datetime.datetime.now()
                    StrCapture = "TIFF " + dt_now.strftime('%Y-%m-%d %H:%M:%S') + " Converted"
                    print(StrCapture)
                    self.iCapRead += 1
                    if(self.iCapRead > 2):
                        self.iCapRead = 1
                    self.fImageReady.set()
                    time.sleep(0.1)
            time.sleep(0.1)

    def CaptureFrames(self):
        if self.fImageReady.is_set():
            while self.fStackBusy == 1:
                time.sleep(0.1)
            self.fStackBusy = 1
            ReadImage = cv2.imread('temp/capture.tiff', -1)
            if self.fDark == 1:
                ReadImage = np.clip((ReadImage.astype(np.float32) - self.DarkImage), 0, 65535)
            self.FrameImage = np.array(ReadImage, dtype=np.uint16)
            self.fImageReady.clear()
            if self.fTrack == 1:
                self.DetectShift()
                if self.fLost == 0:
                    if self.fBaseSet == 0:
                        self.StackImage = self.FrameImage.astype(np.float32)
                        self.StackCounter = 1
                    else:
                        rows, cols, depth = self.FrameImage.shape
                        M = np.float32([[1, 0, -self.ShiftX], [0, 1, -self.ShiftY]])
                        dst = cv2.warpAffine(self.FrameImage, M, (cols, rows))
                        self.StackImage = self.StackImage + dst.astype(np.float32)
                        self.StackCounter += 1
                else:
                    print('TRACK Target Lost or too many stars')
            else:
                floatImage = self.FrameImage.astype(np.float32)
                self.StackImage = self.StackImage + floatImage
                self.StackCounter += 1
            if self.StackCounter > (int(self.MStack) - 1):
                self.MaxStackReached = 1
            self.fStackBusy = 0
            self.globals.fRunDisplayUpdate = 1
            self.globals.fRunZoomUpdate = 1
            dt_now = datetime.datetime.now()
            StrCapture = "BUFFER " + dt_now.strftime('%Y-%m-%d %H:%M:%S') + " Stacked"
            print(StrCapture)
        else:
            time.sleep(0.1)

    def DetectShift(self):
        self.CropImage = self.FrameImage[self.yZoomCenter - self.ZoomWindowHW:self.yZoomCenter + self.ZoomWindowHW,
                                        self.xZoomCenter - self.ZoomWindowHW:self.xZoomCenter + self.ZoomWindowHW]
        self.ZoomImage = (self.CropImage / self.vZAtten).clip(2, 255).astype(np.uint8)
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

    def start_run_thread(self):
        self.stop_run_event.clear()
        self.run_thread = threading.Thread(target=self.run_camera)
        self.run_thread.start()

    def start_convert_thread(self):
        self.stop_convert_event.clear()
        self.convert_thread = threading.Thread(target=self.convert_raw)
        self.convert_thread.start()

    def stop_run_thread(self):
        if self.run_thread:
            self.stop_run_event.set()
            self.run_thread.join(timeout=5)  # タイムアウトを追加
            if self.run_thread.is_alive():
                print("run_thread is still alive, forcing termination")

    def stop_convert_thread(self):
        if self.convert_thread:
            self.stop_convert_event.set()
            self.convert_thread.join(timeout=5)  # タイムアウトを追加
            if self.convert_thread.is_alive():
                print("convert_thread is still alive, forcing termination")


# ==============================================================================
# ファサードエイリアス: MainApp.py から `from CameraController import CameraController`
# のまま使えるように環境に応じたクラスを割り当てる.
# ==============================================================================

_SelectedCameraClass = _auto_select_camera_backend()
CameraController = (_SelectedCameraClass if _SelectedCameraClass is not None 
                    else CameraControllerRaspistill)