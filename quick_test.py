#!/usr/bin/env python3
import sys, os, time, threading, numpy as np, cv2
os.environ['LIBCAMERA_LOG_LEVELS'] = 'FATAL'
sys.path.insert(0, '.')

class FakeMotor:
    def setup_gpio(self): pass
    def cleanup_gpio(self): pass
    TimeStep1=0.02; GuideStep=0.02

class FakeGlobals:
    fThreshold=0; Brightness=0.1; Contrast=0.0
    LevelR=0.11; LevelG=0.1; LevelB=0.114
    Center=0.25; Slope0=0.5; Slope1=10.0
    DispZoomFactor=1; DispScale=32
    fForwardDirection1=1; fRunThread=1
    LoopsToGo2=0; TimeStep2=0.02
    fRunDisplayUpdate=False; fRunZoomUpdate=False

from Picamera2Controller import Picamera2Controller
ctrl = Picamera2Controller(FakeMotor(), FakeGlobals())
ctrl.reset_camera()
print(f"BL={ctrl.black_level} SAT={ctrl.saturation_level}")
ctrl.fRunCamera.set()
ctrl.debug_save_frames = True

# 手動で1フレームキューに投入（capture_request()方式）
req = ctrl.picam2.capture_request()
try:
    raw = req.make_array("raw").copy()
    meta = req.get_metadata().copy()
finally:
    req.release()
print(f"RAW: dtype={raw.dtype} shape={raw.shape}")
ctrl.raw_queue.put((raw, meta))

# convert_raw をスレッドで5秒実行後停止
t = threading.Thread(target=ctrl.convert_raw, daemon=True)
t.start()
time.sleep(5.0)

# 最新PNGを検証
import glob
pngs = sorted(glob.glob('debug_frames/frame_*.png'), key=lambda x: os.path.getmtime(x))
if pngs:
    p = pngs[-1]
    img = cv2.imread(p, cv2.IMREAD_UNCHANGED)
    if img is not None:
        z = 100*np.sum(img==0)/img.size
        print(f"PNG {p}: min={int(img.min())} max={int(img.max())} zeros%={z:.1f}")
        if z < 95:
            print("✅ PASS: 画像データあり")
        else:
            print("❌ FAIL: 真っ黒")
