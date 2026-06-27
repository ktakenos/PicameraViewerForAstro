# GUIHandler.py
import cv2
import numpy as np
import time
from PIL import Image, ImageTk
from tkinter import filedialog


class GUIHandler:
    def __init__(self, camera_controller, stepping_motor_controller, globals):
        self.camera_controller = camera_controller
        self.stepping_motor_controller = stepping_motor_controller
        self.globals = globals

        # メンバー変数の初期化
        self.DisplayImage = None
        self.DisplayR = None
        self.DisplayG = None
        self.DisplayB = None
        self.LUT10bit = np.zeros(1024, dtype=np.uint16)  # Input 10 bits Output 8 bits

        self.Center = float(0.25)
        self.Slope0 = float(0.5)
        self.Slope1 = float(10.0)
        self.MinLUT = np.arctan((self.Slope1**2) * (0 - self.Center))
        self.MaxLUT = np.arctan((self.Slope1**2) * (1.0 - self.Center)) + self.Slope0
        for i in range(1024):
            self.LUT10bit[i] = int(255 * (np.arctan(float(self.Slope1**2) * (float(i) / 1023.0 - float(self.Center))) + float(i) / 1023.0 * float(self.Slope0) - self.MinLUT) / (self.MaxLUT - self.MinLUT))

        # 初期化処理を別々のメソッドに移動
        self.initialize_images()

    def initialize_images(self):
        # 画像配列を初期化
        if self.camera_controller.FrameImage is not None:
            self.DisplayImage = cv2.resize((self.camera_controller.FrameImage / 16).clip(0, 255).astype(np.uint8), 
                                          (int(self.camera_controller.SensorW / 5), int(self.camera_controller.SensorH / 5)))
            self.DisplayR = np.array(self.DisplayImage, dtype=np.float32)
            self.DisplayR = self.DisplayR * 0
            self.DisplayG = np.array(self.DisplayR, dtype=np.float32)
            self.DisplayB = np.array(self.DisplayR, dtype=np.float32)
            self.DisplayB[:, :, 0] = 1.0
            self.DisplayG[:, :, 1] = 1.0
            self.DisplayR[:, :, 2] = 1.0

    def UpdateLUT(self):
        self.MinLUT=np.arctan(float(self.Slope1**2)*(0-float(self.Center)))
        self.MaxLUT=np.arctan(float(self.Slope1**2)*(1.0-float(self.Center)))+float(self.Slope0)
        for i in range(1024):
            self.LUT10bit[i] = int(255*(np.arctan(float(self.Slope1**2)*(float(i)/1023.0-float(self.Center)))+float(i)/1023.0*float(self.Slope0)-self.MinLUT)/(self.MaxLUT-self.MinLUT))
        print("LUT updated")

    def change_threshold(self, vThres):
        self.camera_controller.vThreshold = int(vThres)
        if self.globals.fThreshold == 1:
            self.globals.fRunZoomUpdate = 1

    def change_z_atten(self, ZoomAttenuation, zoomImage):
        self.camera_controller.vZAtten = 10 ** float(0.024082 * int(ZoomAttenuation))
        self.update_zoom(zoomImage)

    def threshold_toggle(self, ThresButton):
        if self.globals.fThreshold == 1:
            self.globals.fThreshold = 0
            ThresButton.configure(bg=self.globals.gray_default)
        else:
            self.globals.fThreshold = 1
            ThresButton.configure(bg=self.globals.ColorBLUE2)
        self.globals.fRunZoomUpdate = 1

    def track_toggle(self, TrackButton):
        if self.camera_controller.fTrack == 1:
            self.camera_controller.fTrack = 0
            self.camera_controller.fBaseSet = 0
            TrackButton.configure(bg=self.globals.gray_default)
        else:
            self.camera_controller.fTrack = 1
            TrackButton.configure(bg=self.globals.ColorBLUE2)

    def dark_toggle(self, DarkButton):
        if self.globals.fDark == 1:
            self.globals.fDark = 0
            DarkButton.configure(bg=self.globals.gray_default)
        else:
            DarkFileName = filedialog.askopenfilename(initialdir='~/Pictures', filetypes=[("Dark File", "*.tif")])
            self.camera_controller.DarkImage = cv2.imread(DarkFileName, -1)  # unchanged
            MinNoise = np.min(self.camera_controller.DarkImage)
            self.camera_controller.DarkImage -= MinNoise
            MinNoise = np.min(self.camera_controller.DarkImage)
            MaxNoise = np.max(self.camera_controller.DarkImage)
            print("DARK-Array    Min=%f, Max=%f" % (MinNoise, MaxNoise))
            self.globals.fDark = 1
            DarkButton.configure(bg=self.globals.ColorBLUE2)

    def change_a_gain(self, AGain):
        self.camera_controller.AnalogGain = int(AGain)

    def change_m_stack(self, MaxStack):
        self.camera_controller.MStack = int(MaxStack)

    def ExposToggle(self, ExposButton):
        if self.camera_controller.fRunCamera.is_set():
            self.camera_controller.fRunCamera.clear()
            ExposButton.configure(bg=self.globals.gray_default)
        else:
            self.camera_controller.fRunCamera.set()
            self.globals.fRunDisplayUpdate = True
            ExposButton.configure(bg=self.globals.ColorBLUE2)
            self.camera_controller.ExpMicSec=int(self.camera_controller.ExpSec * 1000000)

    def change_expos(self, PowOf2, ExpSecLabel):
        self.camera_controller.ExpSec = 2 ** float(PowOf2)
        self.camera_controller.ExpMicSec = int(self.camera_controller.ExpSec * 1000000)
        self.globals.ExpSecStr.set('%1.3f' % self.camera_controller.ExpSec)
        ExpSecLabel.configure(textvariable=self.globals.ExpSecStr)

    def save_stack(self, CountLabel):
        self.globals.fStackBusy = 1
        outfilename = "Pictures/PCIM" + time.strftime("%Y%m%d%H%M%S")
        TIFFilename = outfilename + ".tif"

        MinValue = np.min(self.camera_controller.StackImage)
        MaxValue = np.max(self.camera_controller.StackImage)
        print('Stacked Image Value ranges from %f to %f' % (MinValue, MaxValue))
        FloatImage = self.camera_controller.StackImage - MinValue  # The minimum value shifted to zero
        if int(self.camera_controller.StackCounter) > 0:
            self.camera_controller.StackImage = FloatImage / float(self.camera_controller.StackCounter)  # Averaged
        MinValue = np.min(self.camera_controller.StackImage)
        MaxValue = np.max(self.camera_controller.StackImage)
        print('Stacked Image is averaged')
        print('Stacked Image Value ranges from %f to %f' % (MinValue, MaxValue))
        cv2.imwrite(TIFFilename, self.camera_controller.StackImage)

        print('Stacked Image Saved at ' + time.strftime("%Y%m%d%H%M%S"))
        self.globals.fStackBusy = 0
        self.reset_stack(CountLabel)

    def ResetStack(self, CountLabel):
        self.camera_controller.fStackBusy=1
        self.camera_controller.StackImage=self.camera_controller.StackImage * 0
        self.camera_controller.fStackBusy=0
        self.camera_controller.StackCounter=0
        self.globals.CounterStr.set('%s' % self.camera_controller.StackCounter)
        CountLabel.configure(textvariable = self.globals.CounterStr)

    def stack_show_toggle(self, StackShowButton):
        if self.globals.fStackShow == 1:
            self.globals.fStackShow = 0
            StackShowButton.configure(bg=self.globals.gray_default)
        else:
            self.globals.fStackShow = 1
            StackShowButton.configure(bg=self.globals.ColorBLUE2)
        self.globals.fRunDisplayUpdate = 1

    def level_toggle(self, LevelButton):
        if self.globals.fLevel == 1:
            self.globals.fLevel = 0
            LevelButton.configure(bg=self.globals.gray_default)
        else:
            self.globals.fLevel = 1
            LevelButton.configure(bg=self.globals.ColorBLUE2)
        self.globals.fRunDisplayUpdate = 1

    def reset_level(self, BrightScale):
        self.globals.Brightness = 0
        self.globals.Contrast = 0
        self.globals.fRunDisplayUpdate = 1
        BrightScale.set(self.globals.Brightness)

    def change_brightness(self, ScaleValue):
        self.globals.Brightness = float(ScaleValue)
        self.globals.fRunDisplayUpdate = 1

    def r_level(self, Value):
        self.globals.LevelR = float(Value)
        self.globals.fRunDisplayUpdate = 1

    def g_level(self, Value):
        self.globals.LevelG = float(Value)
        self.globals.fRunDisplayUpdate = 1

    def b_level(self, Value):
        self.globals.LevelB = float(Value)
        self.globals.fRunDisplayUpdate = 1

    def ChangeDispZoom(self, event):
        if(event.delta<0):
            if(self.globals.DispZoomFactor==5):
                self.globals.DispZoomFactor=2
            elif(self.globals.DispZoomFactor==2):
                self.globals.DispZoomFactor=1
        elif(event.delta>0):
            if(self.globals.DispZoomFactor==1):
                self.globals.DispZoomFactor=2
            elif(self.globals.DispZoomFactor==2):
                self.globals.DispZoomFactor=5

    def change_disp_zoom2(self, ScaleValue, CountLabel, lmain, zoomImage):
        self.globals.DispZoomFactor = int(ScaleValue)
        self.update_display(CountLabel, lmain)
        self.update_zoom(zoomImage)

    def update_display(self, CountLabel, lmain):
        if self.DisplayImage is None:
            self.initialize_images()

        Dp = float(self.globals.DispScale)
        X1 = int(self.camera_controller.SensorW / 2) - int(self.camera_controller.SensorW / (2 * int(self.globals.DispZoomFactor)))
        X2 = int(self.camera_controller.SensorW / 2) + int(self.camera_controller.SensorW / (2 * int(self.globals.DispZoomFactor)))
        Y1 = int(self.camera_controller.SensorH / 2) - int(self.camera_controller.SensorH / (2 * int(self.globals.DispZoomFactor)))
        Y2 = int(self.camera_controller.SensorH / 2) + int(self.camera_controller.SensorH / (2 * int(self.globals.DispZoomFactor)))

        self.DisplayImage = cv2.resize(self.camera_controller.FrameImage, (int(self.camera_controller.SensorW / 5), int(self.camera_controller.SensorH / 5)))

        if self.globals.fStackShow == 0:
            if self.globals.DispZoomFactor == 1:
                ResizeImage = np.copy(self.DisplayImage)
            else:
                DispCropImage = self.camera_controller.FrameImage[Y1:Y2, X1:X2]
                ResizeImage = cv2.resize(DispCropImage, (int(self.camera_controller.SensorW / 5), int(self.camera_controller.SensorH / 5)))

            if self.globals.fLevel == 1:
                FloatImage = ResizeImage.astype(np.float32)
                FloatBright = float(self.globals.Brightness)
                FloatImage = FloatImage + FloatBright * 65535
                FloatImage += (self.DisplayR * self.globals.LevelR + self.DisplayG * self.globals.LevelG + self.DisplayB * self.globals.LevelB) * 65535
                FloatImage = np.clip(FloatImage / 64, 0, 1023)
                InputImage = np.uint16(FloatImage)
                self.DisplayImage = np.uint8(self.LUT10bit[InputImage])
            else:
                self.DisplayImage = np.clip((ResizeImage / Dp), 0, 255)

        else:
            if self.camera_controller.StackCounter > 0:
                FloatCounter = float(self.camera_controller.StackCounter)
                self.globals.fStackBusy = 1
                if self.globals.DispZoomFactor == 1:
                    FloatImage = cv2.resize(self.camera_controller.StackImage, (int(self.camera_controller.SensorW / 5), int(self.camera_controller.SensorH / 5)))
                else:
                    DispCropImage = self.camera_controller.StackImage[Y1:Y2, X1:X2]
                    FloatImage = cv2.resize(DispCropImage, (int(self.camera_controller.SensorW / 5), int(self.camera_controller.SensorH / 5)))
                self.globals.fStackBusy = 0

                if self.globals.fLevel == 1:
                    FloatImage = FloatImage / FloatCounter
                    FloatBright = float(self.globals.Brightness)
                    FloatImage = FloatImage + FloatBright * 65535
                    FloatImage += (self.DisplayR * self.globals.LevelR + self.DisplayG * self.globals.LevelG + self.DisplayB * self.globals.LevelB) * 65535
                    FloatImage = np.clip(FloatImage / 64, 0, 1023)
                    InputImage = np.uint16(FloatImage)
                    self.DisplayImage = np.uint8(self.LUT10bit[InputImage])
                else:
                    self.DisplayImage = np.clip((FloatImage / FloatCounter / Dp), 0, 255)
            else:
                self.globals.fStackBusy = 1
                if self.globals.DispZoomFactor == 1:
                    FloatImage = cv2.resize(self.camera_controller.StackImage, (int(self.camera_controller.SensorW / 5), int(self.camera_controller.SensorH / 5)))
                else:
                    DispCropImage = self.camera_controller.StackImage[Y1:Y2, X1:X2]
                    FloatImage = cv2.resize(DispCropImage, (int(self.camera_controller.SensorW / 5), int(self.camera_controller.SensorH / 5)))
                self.globals.fStackBusy = 0
                self.DisplayImage = np.clip((FloatImage / Dp), 0, 255)

        RGBImage = cv2.cvtColor(self.DisplayImage.astype(np.uint8), cv2.COLOR_RGB2BGR)
        img = Image.fromarray(RGBImage)
        imgtk = ImageTk.PhotoImage(image=img)
        lmain.imgtk = imgtk
        lmain.configure(image=imgtk)

        StrAnnot = "Stack %d/%d" % (int(self.camera_controller.StackCounter), int(self.camera_controller.MStack))
        cv2.putText(self.DisplayImage, StrAnnot, (50, int(self.camera_controller.SensorH / 5) - 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.imwrite('/home/pi/temp/display.jpg', self.DisplayImage)

        self.globals.CounterStr.set('%s' % self.camera_controller.StackCounter)
        CountLabel.configure(textvariable=self.globals.CounterStr)

        if(self.camera_controller.MaxStackReached==1):
            self.save_stack()
            self.camera_controller.MaxStackReached=0

    def ZoomPosition(self, event, zoomImage):
        Ex, Ey = event.x, event.y
        x = int(self.camera_controller.SensorW / 2) - int(self.camera_controller.SensorW / (2 * int(self.globals.DispZoomFactor))) + Ex * 5 / int(self.globals.DispZoomFactor)
        y = int(self.camera_controller.SensorH / 2) - int(self.camera_controller.SensorH / (2 * int(self.globals.DispZoomFactor))) + Ey * 5 / int(self.globals.DispZoomFactor)

        if x < self.camera_controller.ZoomWindowHW:
            x = int(self.camera_controller.ZoomWindowHW)
        elif x > self.camera_controller.SensorW - self.camera_controller.ZoomWindowHW:
            x = int(self.camera_controller.SensorW - self.camera_controller.ZoomWindowHW)

        if y < self.camera_controller.ZoomWindowHW:
            y = int(self.camera_controller.ZoomWindowHW)
        elif y > self.camera_controller.SensorH - self.camera_controller.ZoomWindowHW:
            y = int(self.camera_controller.SensorH - self.camera_controller.ZoomWindowHW)

        self.camera_controller.xZoomCenter = int(x)
        self.camera_controller.yZoomCenter = int(y)
        self.globals.fRunZoomUpdate = 1
        self.update_zoom(zoomImage)
        
    def update_zoom(self, zoomImage):
        self.camera_controller.CropImage = self.camera_controller.FrameImage[self.camera_controller.yZoomCenter - self.camera_controller.ZoomWindowHW:self.camera_controller.yZoomCenter + self.camera_controller.ZoomWindowHW,
                             self.camera_controller.xZoomCenter - self.camera_controller.ZoomWindowHW:self.camera_controller.xZoomCenter + self.camera_controller.ZoomWindowHW]
        self.camera_controller.ZoomImage = (self.camera_controller.CropImage / int(self.camera_controller.vZAtten)).clip(2, 255).astype(np.uint8)
        RGBImage = cv2.cvtColor(self.camera_controller.ZoomImage.astype(np.uint8), cv2.COLOR_RGB2BGR)
        GrayFrame = cv2.cvtColor(self.camera_controller.ZoomImage, cv2.COLOR_RGB2GRAY)
        BlurFrame = cv2.blur(GrayFrame, (5, 5))
        ret, GrayFrame0 = cv2.threshold(BlurFrame, int(self.camera_controller.vThreshold), 255, cv2.THRESH_BINARY)
        contours, hierarchy = cv2.findContours(GrayFrame0, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) == 1:
            cnt = contours[0]
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            cv2.circle(GrayFrame0, (int(x), int(y)), int(radius), (255, 255, 255), 2)
            if radius < 32:
                cv2.circle(RGBImage, (int(x), int(y)), int(radius), (0, 255, 0), 2)
            else:
                cv2.circle(RGBImage, (int(x), int(y)), int(radius), (255, 0, 0), 2)

        elif len(contours) > 1:
            cnt = contours[0]
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            cv2.circle(GrayFrame0, (int(x), int(y)), int(radius), (255, 255, 255), 2)
            cv2.circle(RGBImage, (int(x), int(y)), int(radius), (255, 0, 0), 2)

            cnt = contours[1]
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            cv2.circle(GrayFrame0, (int(x), int(y)), int(radius), (255, 255, 255), 2)
            cv2.circle(RGBImage, (int(x), int(y)), int(radius), (255, 0, 0), 2)

        if self.globals.fThreshold == 1:
            img2 = Image.fromarray(GrayFrame0)
        else:
            img2 = Image.fromarray(RGBImage)

        RGBImage = cv2.cvtColor(RGBImage, cv2.COLOR_BGR2RGB)
        cv2.imwrite('/home/pi/temp/zoom.jpg', RGBImage)
        imgtk2 = ImageTk.PhotoImage(image=img2)
        zoomImage.imgtk = imgtk2
        zoomImage.configure(image=imgtk2)

    def reset_stack(self, CountLabel):
        self.globals.fStackBusy = 1
        self.camera_controller.StackImage = self.camera_controller.StackImage * 0
        self.globals.fStackBusy = 0
        self.camera_controller.StackCounter = 0
        self.globals.CounterStr.set('%s' % self.camera_controller.StackCounter)
        CountLabel.configure(textvariable=self.globals.CounterStr)

    def ChangeSlope0(self, ScaleValue):
        self.globals.Slope0 = float(ScaleValue)
        self.UpdateLUT()
        self.globals.fRunDisplayUpdate=1

    def ChangeSlope1(self, ScaleValue):
        self.globals.Slope1 = float(ScaleValue)
        self.UpdateLUT()
        self.globals.fRunDisplayUpdate=1

    def SetTrack(self, SetTrackButton, SetBackwardButton, SetFastforwardButton):
        if(self.camera_controller.fTrack==1):
            return
        self.globals.fForwardDirection1 = 1
        self.stepping_motor_controller.TimeStep1 = self.stepping_motor_controller.GuideStep
        SetTrackButton.configure(bg=self.globals.ColorBLUE2)
        SetBackwardButton.configure(bg=self.globals.gray_default)
        SetFastforwardButton.configure(bg=self.globals.gray_default)

    def SetBackward(self, SetTrackButton, SetBackwardButton, SetFastforwardButton):
        if(self.camera_controller.fTrack==1):
            return
        self.globals.fForwardDirection1 = -1
        self.stepping_motor_controller.TimeStep1 = self.stepping_motor_controller.GuideStep/4.0
        SetBackwardButton.configure(bg=self.globals.ColorBLUE2)
        SetTrackButton.configure(bg=self.globals.gray_default)
        SetFastforwardButton.configure(bg=self.globals.gray_default)

    def SetFastforward(self, SetTrackButton, SetBackwardButton, SetFastforwardButton):
        if(self.camera_controller.fTrack==1):
            return
        self.globals.fForwardDirection1 = 1
        self.stepping_motor_controller.TimeStep1 = self.stepping_motor_controller.GuideStep/5.0
        SetTrackButton.configure(bg=self.globals.gray_default)
        SetBackwardButton.configure(bg=self.globals.gray_default)
        SetFastforwardButton.configure(bg=self.globals.ColorBLUE2)


    def SetLoops2(self, value):   #value in degrees
        self.stepping_motor_controller.LoopsToGo2 = int(float(value)/self.stepping_motor_controller.DegPerStep)

    def GoNorth10(self, GoN10Button):
        if(self.stepping_motor_controller.LoopsToGo2 == 0):
            self.SetLoops2(1.0)
            GoN10Button.configure(bg=self.globals.ColorBLUE2)
        else:
            return

    def GoNorth05(self, GoN05Button):
        if(self.stepping_motor_controller.LoopsToGo2 == 0):
            self.SetLoops2(0.5)
            GoN05Button.configure(bg=self.globals.ColorBLUE2)
        else:
            return

    def GoNorth01(self, GoN01Button):
        if(self.stepping_motor_controller.LoopsToGo2 == 0):
            self.SetLoops2(0.1)
            GoN01Button.configure(bg=self.globals.ColorBLUE2)
        else:
            return

    def StopNS(self, GoN10Button, GoN05Button, GoN01Button, GoS10Button, GoS05Button, GoS01Button):
        self.stepping_motor_controller.LoopsToGo2 = 0
        if(self.stepping_motor_controller.LoopsToGo2 == 0):
            return
        else:
            self.SetLoops2(0)
            GoN10Button.configure(bg=self.globals.gray_default)
            GoN05Button.configure(bg=self.globals.gray_default)
            GoN01Button.configure(bg=self.globals.gray_default)
            GoS01Button.configure(bg=self.globals.gray_default)
            GoS05Button.configure(bg=self.globals.gray_default)
            GoS10Button.configure(bg=self.globals.gray_default)

    def GoSouth01(self, GoS01Button):
        if(self.stepping_motor_controller.LoopsToGo2 == 0):
            self.SetLoops2(-0.1)
            GoS01Button.configure(bg=self.globals.ColorBLUE2)
        else:
            return

    def GoSouth05(self, GoS05Button):
        if(self.stepping_motor_controller.LoopsToGo2 == 0):
            self.SetLoops2(-0.5)
            GoS05Button.configure(bg=self.globals.ColorBLUE2)
        else:
            return

    def GoSouth10(self, GoS10Button):
        if(self.stepping_motor_controller.LoopsToGo2 == 0):
            self.SetLoops2(-1.0)
            GoS10Button.configure(bg=self.globals.ColorBLUE2)
        else:
            return
