# CameraController.py
import subprocess
import threading
import cv2
import numpy as np
import datetime
import time

class CameraController:
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
        self.SaveImage = None
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
        WBGOption = '-r %4.3f %4.3f %4.3f %4.3f' % (self.WBGR, self.WBGG, self.WBGB, self.WBGL)
        StrCommand = 'mv temp/capture1.jpg temp/capture.jpg'
        print(StrCommand)
        res = subprocess.check_call(StrCommand, shell=True)
        dt_now = datetime.datetime.now()
        StrCapture = "Tif pre-conversion at " + dt_now.strftime('%Y-%m-%d %H:%M:%S')
        print(StrCapture)
        StrCommand = 'dcraw -T -4 -q 0 temp/capture.jpg'
        try:
            res = subprocess.check_call(StrCommand, shell=True)
        except subprocess.CalledProcessError as e:
            print(f'Conversion Error: {e}')
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
                    elif self.iCapRead == 2:
                        StrCommand = 'mv temp/capture2.jpg temp/capture.jpg'
                    try:
                        subprocess.check_call(StrCommand, shell=True)
                    except subprocess.CalledProcessError as e:
                        print(f"Error converting raw image: {e}")
                        continue
                    StrCommand = 'dcraw -T -4 -q 0 temp/capture.jpg'
                    try:
                        subprocess.check_call(StrCommand, shell=True)
                    except subprocess.CalledProcessError as e:
                        print(f"Error converting raw image: {e}")
                        continue

                    dt_now = datetime.datetime.now()
                    StrCapture = "TIFF   " + dt_now.strftime('%Y-%m-%d %H:%M:%S') + " Converted"
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
                self.SaveImage = np.copy(self.StackImage)
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