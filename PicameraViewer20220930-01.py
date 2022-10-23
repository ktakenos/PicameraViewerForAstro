# import the necessary packages
import tkinter as tk
from tkinter import filedialog
from PIL import Image, ImageTk
import time
import datetime
import cv2
import numpy as np
import subprocess
import threading
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

GPIO.setup(6,  GPIO.OUT)
GPIO.setup(13, GPIO.OUT)
GPIO.setup(19, GPIO.OUT)
GPIO.setup(26, GPIO.OUT)

GPIO.setup(12, GPIO.OUT)
GPIO.setup(16, GPIO.OUT)
GPIO.setup(20, GPIO.OUT)
GPIO.setup(21, GPIO.OUT)



def ResetCamera():
    global ExpSec,ExpMicSec
    ExpMicSec=int(ExpSec * 1000000)
    
ExpSec=0.25
ExpMicSec=int(ExpSec * 1000000)

#Disabling on-sensor defective pixel correction
#StrCommand = "sudo vcdbg set imx477_dpc 0"#Disabled
#StrCommand = "sudo vcdbg set imx477_dpc 1"#Enabled only Mapped DPC
#StrCommand = "sudo vcdbg set imx477_dpc 2"#Enabled Dynamic DPC
StrCommand = "sudo vcdbg set imx477_dpc 3"#Enabled Both DPC
try:
    res = subprocess.check_call(StrCommand, shell=True)
    print ("DPC is Set")
except:
    print ("DPC is NOT Set")

#capture 2 iamges at startup
Raspistill = 'raspistill -md 3 -ex off -awb off -awbg 1.6,1.7 -drc off -st -t 60 -bm -r -n -q 70 '
AnalogGain = 16
StrCommand = Raspistill + ' -o temp/capture1.jpg -ss ' + str(ExpMicSec) + ' -ag ' + str(AnalogGain)
dt_now=datetime.datetime.now()
StrCapture = "Pre Capture 1 at " + dt_now.strftime('%Y-%m-%d %H:%M:%S')
print(StrCapture)
try:
    res = subprocess.check_call(StrCommand, shell=True)
except:
    print ("Capture Error")
StrCommand = Raspistill + ' -o temp/capture2.jpg -ss ' + str(ExpMicSec) + ' -ag ' + str(AnalogGain)
dt_now=datetime.datetime.now()
StrCapture = "Pre Capture 2 at " + dt_now.strftime('%Y-%m-%d %H:%M:%S')
print(StrCapture)
try:
    res = subprocess.check_call(StrCommand, shell=True)
except:
    print ("Capture Error")

WBGR=1.4
WBGG=1.2
WBGB=1.5
WBGL=1.0
WBGOption='-r %4.3f %4.3f %4.3f %4.3f' % (WBGR, WBGG, WBGB, WBGL)
StrCommand = 'mv temp/capture1.jpg temp/capture.jpg'
res = subprocess.check_call(StrCommand, shell=True)
dt_now=datetime.datetime.now()
StrCapture = "Tif pre-conversion at " + dt_now.strftime('%Y-%m-%d %H:%M:%S')
print(StrCapture)
#StrCommand = 'dcraw %s -T -4 -q 0 temp/capture.jpg' % WBGOption
StrCommand = 'dcraw -T -4 -q 0 temp/capture.jpg'
res = subprocess.check_call(StrCommand, shell=True)

FrameImage = cv2.imread('temp/capture.tiff', -1)
StackImage = np.array(FrameImage*0, dtype = np.float32)
SensorW=4056
SensorH=3040
xZoomCenter=int(SensorW/2)
yZoomCenter=int(SensorH/2)
ZoomWindowHW = 128
CropImage = FrameImage[yZoomCenter-ZoomWindowHW:yZoomCenter+ZoomWindowHW, xZoomCenter-ZoomWindowHW:xZoomCenter+ZoomWindowHW]
#CropImage = FrameImage[yZoomCenter-64:yZoomCenter+64, xZoomCenter-64:xZoomCenter+64]
ZoomImage = (CropImage/16).clip(2,255).astype(np.uint8)

fRunCamera=1
fCapture=2
fCapRead=2
fImageReady=0
def RunCamera():
    global fRunCamera, fCapture, fImageReady
    global ExpMicSec, Raspistill, fCapRead, AnalogGain
    while (fRunCamera):
        if(fCapture==fCapRead):
            if(fCapture==1):
                StrCommand = Raspistill + ' -o temp/capture2.jpg -ss '+str(ExpMicSec) + ' -ag ' + str(AnalogGain)
            elif(fCapture==2):
                StrCommand = Raspistill + ' -o temp/capture1.jpg -ss '+str(ExpMicSec) + ' -ag ' + str(AnalogGain)
            try:
                res = subprocess.check_call(StrCommand, shell=True)
            except:
                print ("Capture Error")
                return

            dt_now=datetime.datetime.now()
            StrCapture = "CAMERA " + dt_now.strftime('%Y-%m-%d %H:%M:%S')
            print("%s Capture%d" % (StrCapture, fCapture))
            
            fCapture+=1
            if(fCapture>2):
                fCapture=1
            time.sleep(0.1)
        time.sleep(0.1)
    print("Camera Stopped")

t1 = threading.Thread(target=RunCamera)
t1.start()

def ConvRaw():
    global fRunCamera, fCapture, fImageReady, fCapRead
    global ExpMicSec, WBGOption
    while (fRunCamera):
        if(fCapture==fCapRead):
            time.sleep(0.1)
            continue
        elif(fImageReady==0):
            fCapRead=fCapture
            if(fCapRead==1):
                StrCommand = 'mv temp/capture1.jpg temp/capture.jpg'
                res = subprocess.check_call(StrCommand, shell=True)
            elif(fCapRead==2):
                StrCommand = 'mv temp/capture2.jpg temp/capture.jpg'
                res = subprocess.check_call(StrCommand, shell=True)
#            StrCommand = 'dcraw %s -T -4 -q 0 temp/capture.jpg' % WBGOption
            StrCommand = 'dcraw -T -4 -q 0 temp/capture.jpg'
            try:
                res = subprocess.check_call(StrCommand, shell=True)
            except:
                print ("Conversion Error")
                return
            
            dt_now=datetime.datetime.now()
            StrCapture = "TIFF   " + dt_now.strftime('%Y-%m-%d %H:%M:%S')+" Converted"
            print(StrCapture)
            
            fImageReady=1
        time.sleep(0.1)
    print("Camera Stopped")

t2 = threading.Thread(target=ConvRaw)
t2.start()



#Set up GUI
window = tk.Tk()  #Makes main window
window.wm_title("PiCamera Viewer")
window.configure(background='#222222')
gray_default=window.cget("background")

ColorRED='#A00'
ColorGREEN='#070'
ColorBLUE='#00A'
ColorBLUE2='#338'

#Graphics window
imageFrame = tk.Frame(window, width=1280, height=720, bg=gray_default,highlightbackground=gray_default)
imageFrame.grid(row=0, column=0, rowspan=3,padx=2, pady=2)
lmain = tk.Label(imageFrame, bg=gray_default)
lmain.grid(row=0, rowspan=10, column=1, columnspan=3)
#JPGImage = cv2.imread('temp/capture.jpg')
DisplayImage = cv2.resize((FrameImage/16).clip(0,255).astype(np.uint8), (int(SensorW/5),int(SensorH/5)))
DisplayR=np.array(DisplayImage, dtype=np.float32)
DisplayR=DisplayR*0
DisplayG=np.array(DisplayR, dtype=np.float32)
DisplayB=np.array(DisplayR, dtype=np.float32)
DisplayB[:,:,0]=1.0
DisplayG[:,:,1]=1.0
DisplayR[:,:,2]=1.0

#RGBImage = cv2.cvtColor(DisplayImage, cv2.COLOR_RGB2BGR)
RGBImage = cv2.cvtColor(DisplayImage.astype(np.uint8), cv2.COLOR_RGB2BGR)
img = Image.fromarray(RGBImage)
imgtk = ImageTk.PhotoImage(image=img)
lmain.imgtk = imgtk
lmain.configure(image=imgtk)

#ZoomImage window 
zoomFrame = tk.Frame(window, width=ZoomWindowHW*2, height=ZoomWindowHW*2, bg=gray_default)
zoomFrame.grid(row = 0, column=1,  padx=2, pady=2)
zoomImage = tk.Label(zoomFrame, bg=gray_default)
zoomImage.grid(row=0, column=0, columnspan=4)
RGBImage = cv2.cvtColor(ZoomImage, cv2.COLOR_RGB2BGR)
img2 = Image.fromarray(RGBImage)
imgtk2 = ImageTk.PhotoImage(image=img2)
zoomImage.imgtk = imgtk2
zoomImage.configure(image=imgtk2)

#For Threshold
fThreshold = 0
vThreshold = 128
def ChangeThres(vThres):
    global vThreshold, fThreshold
    vThreshold = vThres
    if(fThreshold==1):
        global fRunZoomUpdate
        fRunZoomUpdate=1
    #    UpdateZoom()
ThresLabel = tk.Label(zoomFrame, text="Threshold", bg=gray_default, fg='white')
ThresLabel.grid(row=1,column=0,sticky='ew')
ThresScale = tk.Scale(zoomFrame, orient='horizontal', command=ChangeThres,
                      from_=10, to=250, bg=gray_default, fg='white',
                      troughcolor=gray_default,highlightbackground=gray_default,
                      length=100, width=20)
ThresScale.set(vThreshold)
ThresScale.grid(row=1,column=1,sticky='ew')
vZAtten = 16.0
def ChangeZAtten(ZoomAttenuation):
    global vZAtten
    vZAtten = 10 ** float(0.024082*int(ZoomAttenuation))
    UpdateZoom()
ZAttenLabel = tk.Label(zoomFrame, text="Atten", bg=gray_default, fg='white')
ZAttenLabel.grid(row=1,column=2,sticky='ew')
ZAttenScale = tk.Scale(zoomFrame, orient='horizontal', command=ChangeZAtten,
                      from_=0, to=100, resolution=2, bg=gray_default, fg='white',
                      troughcolor=gray_default,highlightbackground=gray_default,
                      length=100, width=20)
ZAttenScale.set(50)
ZAttenScale.grid(row=1,column=3,sticky='ew')
def Threshold():
    global fThreshold
    if(fThreshold==1):
        fThreshold = 0
        ThresButton.configure(bg=gray_default)
    else:
        fThreshold =1
        ThresButton.configure(bg=ColorBLUE2)
    global fRunZoomUpdate
    fRunZoomUpdate=1
#    UpdateZoom()
    UpdateZoom()
ThresButton=tk.Button(zoomFrame, text="Threshold", command =Threshold,
                      bg=gray_default, fg='white')
ThresButton.grid(row=2,column=0,columnspan=1,sticky="ew")

#For Tracking
fTrack = 0
def TrackToggle():
    global fTrack,fBaseSet
    if(fTrack==1):
        fTrack = 0
        fBaseSet = 0
        TrackButton.configure(bg=gray_default)
    else:
        DetectShift()
        fTrack =1
#        fBaseSet = 1
        TrackButton.configure(bg=ColorBLUE2)
TrackButton=tk.Button(zoomFrame, text="Track", command =TrackToggle,
                      bg=gray_default, fg='white')
TrackButton.grid(row=2,column=1,columnspan=1, sticky="ew")

#For Dark
fDark = 0
DarkLevel = 255
DarkFileName=""
DarkFileType=[("Dark File", "*.tif")]
DarkImage = np.float32(FrameImage*0)
def DarkToggle():
    global fDark, DarkFileName, DarkImage, DarkLevel
    if(fDark==1):
        fDark = 0
        DarkButton.configure(bg=gray_default)
    else:
        DarkFileName = filedialog.askopenfilename(initialdir='~/Pictures', filetypes = DarkFileType)
        DarkImage = cv2.imread(DarkFileName, -1)  #unchanged
        MinNoise = np.min(DarkImage)
        DarkImage -= MinNoise
        MinNoise = np.min(DarkImage)
        MaxNoise = np.max(DarkImage)
        print("DARK-Array    Min=%f, Max=%f" % (MinNoise,MaxNoise))
        fDark =1
        DarkButton.configure(bg=ColorBLUE2)
DarkButton=tk.Button(zoomFrame, text="Dark", command =DarkToggle,
                      bg=gray_default, fg='white')
DarkButton.grid(row=2,column=3,columnspan=1, sticky="ew")


#Control Frame
sliderFrame = tk.Frame(window, width=400, height=600, bg=gray_default)
sliderFrame.grid(row = 1, rowspan=3, column=1, padx=2, pady=2)

#For Analog Gain
def ChangeAGain(AGain):
    global AnalogGain
    AnalogGain = AGain
AGainLabel = tk.Label(sliderFrame, text="Analog Gain", bg=gray_default, fg='white')
AGainLabel.grid(row=0,column=0,sticky='ew')
AGainScale = tk.Scale(sliderFrame, orient='horizontal', command=ChangeAGain,
                      from_=1, to=16, bg=gray_default, fg='white',
                      troughcolor=gray_default,highlightbackground=gray_default,
                      length=100, width=20)
AGainScale.set(AnalogGain)
AGainScale.grid(row=0,column=1,sticky='ew')

#For Max Stack Adjustment
MStack=64
def ChangeMStack(MaxStack):
    global MStack
    MStack = MaxStack
MStackLabel = tk.Label(sliderFrame, text="MaxStack", bg=gray_default, fg='white')
MStackLabel.grid(row=0,column=2,sticky='ew')
MStackScale = tk.Scale(sliderFrame, orient='horizontal', command=ChangeMStack,
                      from_=16, to=128, bg=gray_default, fg='white',
                      troughcolor=gray_default,highlightbackground=gray_default,
                      length=100, width=20)
MStackScale.set(MStack)
MStackScale.grid(row=0,column=3,sticky='ew')


fRunImageUpdate=0
def ExposToggle():
    global ExpSec,fRunImageUpdate
    if(fRunImageUpdate==1):
        fRunImageUpdate = 0
        ExposButton.configure(bg=gray_default)
    else:
        fRunImageUpdate =1
        ExposButton.configure(bg=ColorBLUE2)
        ResetCamera()
#        CaptureFrames()
ExposButton=tk.Button(sliderFrame, text="Capture", command =ExposToggle, bg=gray_default, fg='white')
ExposButton.grid(row=5,column=0, sticky="ew")


#For Exposure
ExpSecStr=tk.StringVar()
PowerOf2=-2
def ChangeExpos(PowOf2):
    global ExpSec,ExpMicSec
    ExpSec = 2 ** float(PowOf2)
    ExpMicSec=int(ExpSec * 1000000)
    ExpSecStr.set('%1.3f' % ExpSec)
    ExpSecLabel.configure(textvariable = ExpSecStr)
ExposLabel = tk.Label(sliderFrame, text="Exposure[s]", bg=gray_default, fg='white')
ExposLabel.grid(row=5,column=1,sticky='ew')
ExpSecStr.set('%1.3f' % ExpSec)
ExpSecLabel = tk.Label(sliderFrame, textvariable=ExpSecStr, bg=gray_default, fg='white')
ExpSecLabel.grid(row=5,column=2,sticky='ew')
ExposScale = tk.Scale(sliderFrame, orient='horizontal', command=ChangeExpos,
                      from_=-8, to=5, bg=gray_default, fg='white', resolution=0.5,
                      troughcolor=gray_default,highlightbackground=gray_default,
                      length=100, width=20)
ExposScale.set(PowerOf2)
ExposScale.grid(row=5,column=3,sticky='ew')

#For Stack Reset
StackCounter=0
CounterStr=tk.StringVar()
CounterStr.set('%s' % StackCounter)
StackLabel = tk.Label(sliderFrame, text="Stack", bg=gray_default, fg='white')
StackLabel.grid(row=7,column=0,sticky='ew')
CountLabel = tk.Label(sliderFrame, textvariable=CounterStr, bg=gray_default, fg='white')
CountLabel.grid(row=7,column=1,sticky='ew')
def ResetStack():
    global StackImage, StackCounter, fStackBusy
    fStackBusy=1
    StackImage=StackImage*0
    fStackBusy=0
    StackCounter=0
    CounterStr.set('%s' % StackCounter)
    CountLabel.configure(textvariable = CounterStr)
ResetButton=tk.Button(sliderFrame, text="Reset", command =ResetStack, bg=gray_default, fg='white')
ResetButton.grid(row=7,column=3,sticky="ew")

#For Stack Save
def SaveStack():
    global StackImage, StackCounter, fStackBusy, MStack
    fStackBusy=1
    outfilename="Pictures/PCIM"+time.strftime("%Y%m%d%H%M%S")
    TIFFilename=outfilename+".tif"

    MinValue = np.min(StackImage)
    MaxValue = np.max(StackImage)
    print('Stacked Image Value ranges from %f to %f' %  (MinValue,MaxValue))
    FloatImage = StackImage - MinValue #The minimum value shifted to zero
    if(int(StackCounter)>0):
        StackImage = FloatImage/float(StackCounter) #Averaged
    MinValue = np.min(StackImage)
    MaxValue = np.max(StackImage)
    print('Stacked Image is averaged')
    print('Stacked Image Value ranges from %f to %f' %  (MinValue,MaxValue))
    cv2.imwrite(TIFFilename, StackImage)

    print('Stacked Image Saved at '+ time.strftime("%Y%m%d%H%M%S"))
    fStackBusy=0
    ResetStack()
SaveButton=tk.Button(sliderFrame, text="Save", command =SaveStack, bg=gray_default, fg='white')
SaveButton.grid(row=7,column=2, sticky="ew")

#For Stack Show
fStackShow = 0
def StackShowToggle():
    global fStackShow
    if(fStackShow==1):
        fStackShow = 0
        StackShowButton.configure(bg=gray_default)
    else:
        fStackShow =1
        StackShowButton.configure(bg=ColorBLUE2)
    global fRunDisplayUpdate
    fRunDisplayUpdate=1
#    UpdateDisplay()
StackShowButton=tk.Button(sliderFrame, text="Show Stack", command =StackShowToggle, bg=gray_default, fg='white')
StackShowButton.grid(row=9,column=0,sticky="ew")

#For Level Adjust
fLevel = 0
def LevelToggle():
    global fLevel
    if(fLevel==1):
        fLevel = 0
        LevelButton.configure(bg=gray_default)
    else:
        fLevel =1
        LevelButton.configure(bg=ColorBLUE2)
    global fRunDisplayUpdate
    fRunDisplayUpdate=1
#    UpdateDisplay()
LevelButton=tk.Button(sliderFrame, text="Level Adjust", command =LevelToggle, bg=gray_default, fg='white')
LevelButton.grid(row=9,column=1,columnspan=2,sticky="ew")

#For Reset Level
Contrast=0.0
def ResetLevel():
    global Brightness
    global Contrast
    Brightness=0
    Contrast=0
    global fRunDisplayUpdate
    fRunDisplayUpdate=1
#    UpdateDisplay()
    BrightScale.set(Brightness)
    ContrScale.set(Contrast)
ResetLevelButton=tk.Button(sliderFrame, text="Reset Level", command =ResetLevel, bg=gray_default, fg='white')
ResetLevelButton.grid(row=9,column=3,sticky="ew")

#For Brightness
Brightness=0.1
def ChangeBrightness(ScaleValue):
    global Brightness
    Brightness = float(ScaleValue)
    global fRunDisplayUpdate
    fRunDisplayUpdate=1
#    UpdateDisplay()
BrightLabel = tk.Label(sliderFrame, text="Level", bg=gray_default, fg='white')
BrightLabel.grid(row=10,column=0,sticky='ew')
BrightScale = tk.Scale(sliderFrame, orient='horizontal', command=ChangeBrightness,
                       from_=0, to=0.5, bg=gray_default, fg='white',
                       resolution = 0.0005,
                       troughcolor=gray_default,highlightbackground=gray_default,
                      length=100, width=20)
BrightScale.set(Brightness)
BrightScale.grid(row=11,column=0,sticky='ew')

#For Dark Color Balance
LevelR=0.11
LevelG=0.1
LevelB=0.114
def RLevel(Value):
    global LevelR
    LevelR=float(Value)
    global fRunDisplayUpdate
    fRunDisplayUpdate=1
RLevelLabel = tk.Label(sliderFrame, text="RED", bg=gray_default, fg='white')
RLevelLabel.grid(row=10,column=1,sticky='ew')
RLevelScale = tk.Scale(sliderFrame, orient='horizontal', command=RLevel,
                      from_=0, to=0.2, bg=gray_default, fg='white',
                       resolution=0.0005,
                      troughcolor=gray_default,highlightbackground=gray_default,
                      length=100, width=20)
RLevelScale.set(LevelR)
RLevelScale.grid(row=11,column=1,sticky='ew')
def GLevel(Value):
    global LevelG
    LevelG=float(Value)
    global fRunDisplayUpdate
    fRunDisplayUpdate=1
GLevelLabel = tk.Label(sliderFrame, text="GREEN", bg=gray_default, fg='white')
GLevelLabel.grid(row=10,column=2,sticky='ew')
GLevelScale = tk.Scale(sliderFrame, orient='horizontal', command=GLevel,
                      from_=0, to=0.2, bg=gray_default, fg='white',
                       resolution=0.0005,
                      troughcolor=gray_default,highlightbackground=gray_default,
                      length=100, width=20)
GLevelScale.set(LevelG)
GLevelScale.grid(row=11,column=2,sticky='ew')
def BLevel(Value):
    global LevelB
    LevelB=float(Value)
    global fRunDisplayUpdate
    fRunDisplayUpdate=1
BLevelLabel = tk.Label(sliderFrame, text="BLUE", bg=gray_default, fg='white')
BLevelLabel.grid(row=10,column=3,sticky='ew')
BLevelScale = tk.Scale(sliderFrame, orient='horizontal', command=BLevel,
                      from_=0, to=0.2, bg=gray_default, fg='white',
                       resolution=0.0005,
                      troughcolor=gray_default,highlightbackground=gray_default,
                      length=100, width=20)
BLevelScale.set(LevelB)
BLevelScale.grid(row=11,column=3,sticky='ew')

#For Contrast
ContrLabel = tk.Label(sliderFrame, text="Contrast", bg=gray_default, fg='white')
ContrLabel.grid(row=12,column=0,sticky='ew')

LUT10bit = np.zeros(1024, dtype = np.uint16)#Input 10 bits Output 8 bits
Center=float(0.25)
Slope0=float(0.5)
Slope1=float(10.0)
MinLUT=np.arctan((Slope1**2)*(0-Center))
MaxLUT=np.arctan((Slope1**2)*(1.0-Center))+Slope0
for i in range(1024):
    LUT10bit[i] = int(255*(np.arctan(float(Slope1**2)*(float(i)/1023.0-float(Center)))+float(i)/1023.0*float(Slope0)-MinLUT)/(MaxLUT-MinLUT))
def UpdateLUT():
    global LUT10bit, Center, Slope0, Slope1, MinLUT, MaxLUT
    MinLUT=np.arctan(float(Slope1**2)*(0-float(Center)))
    MaxLUT=np.arctan(float(Slope1**2)*(1.0-float(Center)))+float(Slope0)
    for i in range(1024):
        LUT10bit[i] = int(255*(np.arctan(float(Slope1**2)*(float(i)/1023.0-float(Center)))+float(i)/1023.0*float(Slope0)-MinLUT)/(MaxLUT-MinLUT))
    print("LUT updated")

def ChangeSlope0(ScaleValue):
    global Slope0
    Slope0 = float(ScaleValue)
    UpdateLUT()
    global fRunDisplayUpdate
    fRunDisplayUpdate=1
Slope0Scale = tk.Scale(sliderFrame, orient='horizontal', command=ChangeSlope0,
                      from_=0, to=1.0, bg=gray_default, fg='white',
                       resolution=0.01,
                      troughcolor=gray_default,highlightbackground=gray_default,
                      length=100, width=20)
Slope0Scale.set(Slope0)
Slope0Scale.grid(row=12,column=1,sticky='ew')
def ChangeSlope1(ScaleValue):
    global Slope1
    Slope1 = float(ScaleValue)
    UpdateLUT()
    global fRunDisplayUpdate
    fRunDisplayUpdate=1
Slope1Scale = tk.Scale(sliderFrame, orient='horizontal', command=ChangeSlope1,
                      from_=1, to=30, bg=gray_default, fg='white',
                       resolution=0.5,
                      troughcolor=gray_default,highlightbackground=gray_default,
                      length=100, width=20)
Slope1Scale.set(Slope1)
Slope1Scale.grid(row=12,column=2,sticky='ew')


DispZoomFactor=1
def ChangeDispZoom(event):
    global DispZoomFactor
    print("MouseWheel Event Callback is called")
    if(event.delta<0):
        if(DispZoomFactor==5):
            DispZoomFactor=2
        elif(DispZoomFactor==2):
            DispZoomFactor=1
        print("MouseWheel Down")
    elif(event.delta>0):
        if(DispZoomFactor==1):
            DispZoomFactor=2
        elif(DispZoomFactor==2):
            DispZoomFactor=5
        print("MouseWheel Up")
lmain.bind('<MouseWheel>', ChangeDispZoom)

def ChangeDispZoom2(ScaleValue):
    global DispZoomFactor
    DispZoomFactor=int(ScaleValue)
    UpdateDisplay()
    UpdateZoom()
DZScale = tk.Scale(imageFrame, orient='vertical', command=ChangeDispZoom2,
                      from_=1, to=5, bg=gray_default, fg='white', resolution=1,
                      troughcolor=gray_default,highlightbackground=gray_default,
                      length=50, width=20)
DZScale.set(DispZoomFactor)
DZScale.grid(row=0,column=0,sticky='ns')


DispScale=32
def UpdateDisplay():
    global FrameImage, StackImage, fLevel, fStackShow, StackCounter, fStackBusy
    global SensorW, SensorH
    global Brightness, ContrV, MStack, LUT10bit
    global DispScale, DispZoomFactor
    global DisplayR, DisplayG, DisplayB, LevelR, LevelG, LevelB
    Dp=float(DispScale)
    X1=int(SensorW/2)-int(SensorW/(2*int(DispZoomFactor)))
    X2=int(SensorW/2)+int(SensorW/(2*int(DispZoomFactor)))
    Y1=int(SensorH/2)-int(SensorH/(2*int(DispZoomFactor)))
    Y2=int(SensorH/2)+int(SensorH/(2*int(DispZoomFactor)))
    if(fStackShow==0):
        if(DispZoomFactor==1):
            ResizeImage = cv2.resize(FrameImage, (int(SensorW/5),int(SensorH/5)))
        else:
            CropImage = FrameImage[Y1:Y2, X1:X2]
            ResizeImage = cv2.resize(CropImage, (int(SensorW/5),int(SensorH/5)))
        if(fLevel==1):
            FloatImage = ResizeImage.astype(np.float32)
            FloatBright=float(Brightness)
            FloatImage = FloatImage+FloatBright*65535
            FloatImage += (DisplayR*LevelR +DisplayG*LevelG +DisplayB*LevelB)*65535 
            FloatImage = np.clip(FloatImage/64, 0, 1023)
            InputImage = np.uint16(FloatImage)
            DisplayImage = np.uint8(LUT10bit[InputImage])
        else:
            DisplayImage = np.clip((ResizeImage/Dp), 0, 255)
    else:
        if(StackCounter>0):
            FloatCounter=float(StackCounter)
            fStackBusy=1
            if(DispZoomFactor==1):
                FloatImage = cv2.resize(StackImage, (int(SensorW/5),int(SensorH/5)))
            else:
                CropImage = StackImage[Y1:Y2, X1:X2]
                FloatImage = cv2.resize(CropImage, (int(SensorW/5),int(SensorH/5)))
            fStackBusy=0
            if(fLevel==1):
                FloatImage = FloatImage/FloatCounter
                FloatBright=float(Brightness)
                FloatImage = FloatImage+FloatBright*65535
                FloatImage += (DisplayR*LevelR +DisplayG*LevelG +DisplayB*LevelB)*65535 
                FloatImage = np.clip(FloatImage/64, 0, 1023)
                InputImage = np.uint16(FloatImage)
                DisplayImage = np.uint8(LUT10bit[InputImage])
            else:
                DisplayImage = np.clip((FloatImage/FloatCounter/Dp), 0, 255)
        else:
            fStackBusy=1
            if(DispZoomFactor==1):
                FloatImage = cv2.resize(StackImage, (int(SensorW/5),int(SensorH/5)))
            else:
                CropImage = StackImage[Y1:Y2, X1:X2]
                FloatImage = cv2.resize(CropImage, (int(SensorW/5),int(SensorH/5)))
            fStackBusy=0
            DisplayImage = np.clip((FloatImage/Dp), 0, 255)
    RGBImage = cv2.cvtColor(DisplayImage.astype(np.uint8), cv2.COLOR_RGB2BGR)
    StrAnnot = "Stack %d/%d" % (int(StackCounter), int(MStack))
    cv2.putText(DisplayImage, StrAnnot, (50, int(SensorH/5)-50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
    cv2.imwrite('/home/pi/temp/display.jpg', DisplayImage)
    img = Image.fromarray(RGBImage)
    imgtk = ImageTk.PhotoImage(image=img)
    lmain.imgtk = imgtk
    lmain.configure(image=imgtk)

def ZoomPosition(event):
    global xZoomCenter, yZoomCenter, SensorW, SensorH, ZoomWindowHW
    global fTrack, DispZoomFactor
    if(fTrack==1):
        return
    Ex, Ey = event.x, event.y

    x=int(SensorW/2)-int(SensorW/(2*int(DispZoomFactor)))+Ex*5/int(DispZoomFactor)
    y=int(SensorH/2)-int(SensorH/(2*int(DispZoomFactor)))+Ey*5/int(DispZoomFactor)
    
    if(x<ZoomWindowHW):
        x=int(ZoomWindowHW)
    elif(x>SensorW-ZoomWindowHW):
        x=int(SensorW-ZoomWindowHW)
    if(y<ZoomWindowHW):
        y=int(ZoomWindowHW)
    elif(y>SensorH-ZoomWindowHW):
        y=int(SensorH-ZoomWindowHW)
    xZoomCenter=int(x)
    yZoomCenter=int(y)
    global fRunZoomUpdate
    fRunZoomUpdate=1
lmain.bind('<ButtonPress-1>', ZoomPosition)

def UpdateZoom():
    global ZoomImage, xZoomCenter, yZoomCenter, FrameImage, ZoomImage, vThreshold, fThreshold, ZoomWindowHW
    global vZAtten
    CropImage = FrameImage[yZoomCenter-ZoomWindowHW:yZoomCenter+ZoomWindowHW, xZoomCenter-ZoomWindowHW:xZoomCenter+ZoomWindowHW]
    ZoomImage = (CropImage/int(vZAtten)).clip(2,255).astype(np.uint8)
    RGBImage = cv2.cvtColor(ZoomImage.astype(np.uint8), cv2.COLOR_RGB2BGR)
    GrayFrame=cv2.cvtColor(ZoomImage,cv2.COLOR_RGB2GRAY)
    BlurFrame=cv2.blur(GrayFrame,(5,5))
    ret,GrayFrame0 = cv2.threshold(BlurFrame, int(vThreshold), 255, cv2.THRESH_BINARY)
    contours,hierarchy = cv2.findContours(GrayFrame0, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if (len(contours) == 1):
        cnt = contours[0]
        (x,y),radius = cv2.minEnclosingCircle(cnt)
        cv2.circle(GrayFrame0, (int(x),int(y)), int(radius), (255,255,255),2)
        if(radius<32):
            cv2.circle(RGBImage, (int(x),int(y)), int(radius), (0,255,0),2)
        else:
            cv2.circle(RGBImage, (int(x),int(y)), int(radius), (255,0,0),2)
    elif(len(contours) > 1):
        cnt = contours[0]
        (x,y),radius = cv2.minEnclosingCircle(cnt)
        cv2.circle(GrayFrame0, (int(x),int(y)), int(radius), (255,255,255),2)
        cv2.circle(RGBImage, (int(x),int(y)), int(radius), (255,0,0),2)
        cnt = contours[1]
        (x,y),radius = cv2.minEnclosingCircle(cnt)
        cv2.circle(GrayFrame0, (int(x),int(y)), int(radius), (255,255,255),2)
        cv2.circle(RGBImage, (int(x),int(y)), int(radius), (255,0,0),2)
    if(fThreshold == 1):
        img2 = Image.fromarray(GrayFrame0)
    else:
        img2 = Image.fromarray(RGBImage)
    RGBImage = cv2.cvtColor(RGBImage, cv2.COLOR_BGR2RGB)
    cv2.imwrite('/home/pi/temp/zoom.jpg', RGBImage)
    imgtk2 = ImageTk.PhotoImage(image=img2)
    zoomImage.imgtk = imgtk2
    zoomImage.configure(image=imgtk2)

BaseX = xZoomCenter
BaseY = yZoomCenter
ShiftX = 0
ShiftY = 0
fBaseSet=0
fLost=0
def DetectShift():
    global fBaseSet, BaseX, BaseY, ShiftX, ShiftY, fLost
    global xZoomCenter, yZoomCenter, FrameImage, ZoomImage, vThreshold, fThreshold, ZoomWindowHW
    global vZAtten
    global TimeStep1, GuideStep
    CropImage = FrameImage[yZoomCenter-ZoomWindowHW:yZoomCenter+ZoomWindowHW, xZoomCenter-ZoomWindowHW:xZoomCenter+ZoomWindowHW]
    ZoomImage = (CropImage/int(vZAtten)).clip(2,255).astype(np.uint8)
    GrayFrame=cv2.cvtColor(ZoomImage,cv2.COLOR_RGB2GRAY)
    BlurFrame=cv2.blur(GrayFrame,(5,5))
    ret,GrayFrame0 = cv2.threshold(BlurFrame, int(vThreshold), 255, cv2.THRESH_BINARY)
    contours,hierarchy = cv2.findContours(GrayFrame0, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if (len(contours) == 1):
        cnt = contours[0]
        (x,y),radius = cv2.minEnclosingCircle(cnt)
        if(radius < 32):
            if(fBaseSet==0):
                BaseX = int(x)+xZoomCenter-ZoomWindowHW
                BaseY = int(y)+yZoomCenter-ZoomWindowHW
                #Recentering
                xZoomCenter += (int(x)-ZoomWindowHW)
                yZoomCenter += (int(y)-ZoomWindowHW)
                fBaseSet=1
            else:
                ShiftX = int(x)+xZoomCenter-ZoomWindowHW - BaseX
                ShiftY = int(y)+yZoomCenter-ZoomWindowHW - BaseY
                xZoomCenter += int(x)-ZoomWindowHW
                yZoomCenter += int(y)-ZoomWindowHW
                if(ShiftX> 5): #Positive = Star moves faster so speeding up
                    TimeStep1 = GuideStep*0.8
                elif(ShiftX < -5):#Negative = star moves slowly so speeding down
                    TimeStep1 = GuideStep*1.2
                if(ShiftY > 5): 
                    SetLoops2(-0.0026) 
                elif(ShiftY < -5): 
                    SetLoops2(0.0026) 
            fLost = 0
        else:
            fLost=1
    else:
        fLost=1

# allow the camera to warmup
time.sleep(0.5)
mSecSleep = 20
fStackBusy = 0
def CaptureFrames():
    global fCapture, rawCapture, FrameImage, StackImage, StackCounter, ZoomImage, DisplayImage
    global fBaseSet, BaseX, BaseY, ShiftX, ShiftY, fLost
    global ExpMicSec, fImageReady, fStackBusy, MStack
    global fDark, DarkFileName, DarkImage, DarkLevel
    if(fImageReady==1):
        while(fStackBusy==1):
            time.sleep(0.1)
        fStackBusy=1
        ReadImage = cv2.imread('temp/capture.tiff', -1)
        if(fDark==1):
            ReadImage = np.clip((ReadImage.astype(np.float32)-DarkImage), 0, 65535)
        FrameImage = np.array(ReadImage, dtype=np.uint16)
        fImageReady=0
        if(fTrack==1):
            DetectShift()
            if(fLost==0):
                if(fBaseSet==0):
                    StackImage = FrameImage.astype(np.float32)
                    StackCounter = 1
                else:
                    rows,cols,depth = FrameImage.shape
                    M = np.float32([[1,0,-ShiftX],[0,1,-ShiftY]])
                    dst = cv2.warpAffine(FrameImage,M,(cols,rows))
                    StackImage = StackImage + dst.astype(np.float32)
                    StackCounter += 1
                TrackButton.configure(bg=ColorBLUE2)
            else:
                TrackButton.configure(bg=ColorRED)
                print('TRACK  Target Lost or too many stars')
        else:
            floatImage = FrameImage.astype(np.float32)
            StackImage = StackImage + floatImage
            StackCounter += 1
        CounterStr.set('%s' % StackCounter)
        CountLabel.configure(textvariable = CounterStr)
        if(StackCounter>(int(MStack)-1)):
            SaveStack()
        fStackBusy=0
        global fRunDisplayUpdate,fRunZoomUpdate
        fRunDisplayUpdate=1
        fRunZoomUpdate=1
        dt_now=datetime.datetime.now()
        StrCapture = "BUFFER " + dt_now.strftime('%Y-%m-%d %H:%M:%S') + " Stacked"
        print(StrCapture)
        
            
def RunImageUpdate():
    global fRunImageUpdate, fCapture, fImageReady
    while (1):
        if(fRunImageUpdate):
            CaptureFrames()
        time.sleep(0.1)

time.sleep(1)
t3 = threading.Thread(target=RunImageUpdate)
t3.start()

fRunDisplayUpdate=1
fRunZoomUpdate=1
def RunDisplayUpdate():
    global fRunDisplayUpdate,fRunZoomUpdate
    while (1):
        if(fRunDisplayUpdate):
            UpdateDisplay()
            fRunDisplayUpdate=0
        if(fRunZoomUpdate):
            UpdateZoom()
            fRunZoomUpdate=0
        time.sleep(0.2)

t4 = threading.Thread(target=RunDisplayUpdate)
t4.start()

#Ratio inside stepper
StepPerRev = 32
RatioInGear = 64
RatioOutGear = 4
DegWarm = 4
DegPerStep = DegWarm/(RatioOutGear * RatioInGear * StepPerRev)
DegPerSec = 360/(24 * 60 * 60)
GuideStep = DegPerStep / DegPerSec

fForwardDirection1 = 1
fRunThread = 1
TimeStep1 = GuideStep
LoopsToGo2 = 0
TimeStep2 = 0.02

def SetTrack():
    global fTrack
    if(fTrack==1):
        return
    global TimeStep1, fForwardDirection1
    fForwardDirection1 = 1
    TimeStep1 = GuideStep
    SetTrackButton.configure(bg=ColorBLUE2)
    SetBackwardButton.configure(bg=gray_default)
    SetFastforwardButton.configure(bg=gray_default)
SetTrackButton=tk.Button(imageFrame, text="Normal", command =SetTrack, bg=gray_default, fg='white')
SetTrackButton.grid(row=10,column=2,sticky="ew")
SetTrackButton.configure(bg=ColorBLUE2)
def SetBackward():
    global fTrack
    if(fTrack==1):
        return
    global TimeStep1, fForwardDirection1
    fForwardDirection1 = -1
    TimeStep1 = GuideStep/4.0
    SetBackwardButton.configure(bg=ColorBLUE2)
    SetTrackButton.configure(bg=gray_default)
    SetFastforwardButton.configure(bg=gray_default)
SetBackwardButton=tk.Button(imageFrame, text="<<5x", command =SetBackward, bg=gray_default, fg='white')
SetBackwardButton.grid(row=10,column=1,sticky="ew")
def SetFastforward():
    global fTrack
    if(fTrack==1):
        return
    global TimeStep1, fForwardDirection1
    fForwardDirection1 = 1
    TimeStep1 = GuideStep/5.0
    SetTrackButton.configure(bg=gray_default)
    SetBackwardButton.configure(bg=gray_default)
    SetFastforwardButton.configure(bg=ColorBLUE2)
SetFastforwardButton=tk.Button(imageFrame, text="5x>>", command =SetFastforward, bg=gray_default, fg='white')
SetFastforwardButton.grid(row=10,column=3,sticky="ew")


def SetLoops2(value):   #value in degrees
    global LoopsToGo2, DegPerStep
    LoopsToGo2 = int(float(value)/DegPerStep)

NorthLabel = tk.Label(imageFrame, text="N", bg=gray_default, fg='white')
NorthLabel.grid(row=1,column=0,sticky='ew')
def GoNorth10():
    global LoopsToGo2    
    if(LoopsToGo2 == 0):
        SetLoops2(1.0)
        GoN10Button.configure(bg=ColorBLUE2)
    else:
        return
GoN10Button=tk.Button(imageFrame, text="1.0", command =GoNorth10, bg=gray_default, fg='white')
GoN10Button.grid(row=2,column=0,sticky="ew")
def GoNorth05():
    global LoopsToGo2    
    if(LoopsToGo2 == 0):
        SetLoops2(0.5)
        GoN05Button.configure(bg=ColorBLUE2)
    else:
        return
GoN05Button=tk.Button(imageFrame, text="0.5", command =GoNorth05, bg=gray_default, fg='white')
GoN05Button.grid(row=3,column=0,sticky="ew")
def GoNorth01():
    global LoopsToGo2    
    if(LoopsToGo2 == 0):
        SetLoops2(0.1)
        GoN01Button.configure(bg=ColorBLUE2)
    else:
        return
GoN01Button=tk.Button(imageFrame, text="0.1", command =GoNorth01, bg=gray_default, fg='white')
GoN01Button.grid(row=4,column=0,sticky="ew")

def StopNS():
    global LoopsToGo2    
    if(LoopsToGo2 == 0):
        return
    else:
        SetLoops2(0)
        GoN10Button.configure(bg=gray_default)
        GoN05Button.configure(bg=gray_default)
        GoN01Button.configure(bg=gray_default)
        GoS01Button.configure(bg=gray_default)
        GoS05Button.configure(bg=gray_default)
        GoS10Button.configure(bg=gray_default)
StopNSButton=tk.Button(imageFrame, text="0", command =StopNS, bg=gray_default, fg='white')
StopNSButton.grid(row=5,column=0,sticky="ew")


def GoSouth01():
    global LoopsToGo2    
    if(LoopsToGo2 == 0):
        SetLoops2(-0.1)
        GoS01Button.configure(bg=ColorBLUE2)
    else:
        return
GoS01Button=tk.Button(imageFrame, text="0.1", command =GoSouth01, bg=gray_default, fg='white')
GoS01Button.grid(row=6,column=0,sticky="ew")
def GoSouth05():
    global LoopsToGo2    
    if(LoopsToGo2 == 0):
        SetLoops2(-0.5)
        GoS05Button.configure(bg=ColorBLUE2)
    else:
        return
GoS05Button=tk.Button(imageFrame, text="0.5", command =GoSouth05, bg=gray_default, fg='white')
GoS05Button.grid(row=7,column=0,sticky="ew")
def GoSouth10():
    global LoopsToGo2    
    if(LoopsToGo2 == 0):
        SetLoops2(-1.0)
        GoS10Button.configure(bg=ColorBLUE2)
    else:
        return
GoS10Button=tk.Button(imageFrame, text="1.0", command =GoSouth10, bg=gray_default, fg='white')
GoS10Button.grid(row=8,column=0,sticky="ew")
SouthLabel = tk.Label(imageFrame, text="S", bg=gray_default, fg='white')
SouthLabel.grid(row=9,column=0,sticky='ew')


nPhase1 = 0
def PortOutTh1():
    global fRunThread, nPhase1, fForwardDirection1, TimeStep1
    while (fRunThread == 1):
        if(fForwardDirection1 ==1): #Positive value goes west
            if(nPhase1 == 0):
                GPIO.output(6,  1)
                GPIO.output(13, 0)
                GPIO.output(19, 0)
                GPIO.output(26, 0)
                nPhase1=1
            elif(nPhase1 == 1):
                GPIO.output(13, 1)
                GPIO.output(19, 0)
                GPIO.output(26, 0)
                GPIO.output(6,  0)
                nPhase1=2
            elif(nPhase1 == 2):
                GPIO.output(19, 1)
                GPIO.output(26, 0)
                GPIO.output(6,  0)
                GPIO.output(13, 0)
                nPhase1=3
            elif(nPhase1 == 3):
                GPIO.output(26, 1)
                GPIO.output(6,  0)
                GPIO.output(13, 0)
                GPIO.output(19, 0)
                nPhase1=0
        elif(fForwardDirection1 == -1):
            if(nPhase1 == 0):
                GPIO.output(6,  1)
                GPIO.output(26, 0)
                GPIO.output(19, 0)
                GPIO.output(13, 0)
                nPhase1=3
            elif(nPhase1 == 3):
                GPIO.output(26, 1)
                GPIO.output(19, 0)
                GPIO.output(13, 0)
                GPIO.output(6,  0)
                nPhase1=2
            elif(nPhase1 == 2):
                GPIO.output(19, 1)
                GPIO.output(13, 0)
                GPIO.output(6,  0)
                GPIO.output(26, 0)
                nPhase1=1
            elif(nPhase1 == 1):
                GPIO.output(13, 1)
                GPIO.output(6,  0)
                GPIO.output(26, 0)
                GPIO.output(19, 0)
                nPhase1=0
        time.sleep(TimeStep1)
t5 = threading.Thread(target=PortOutTh1)
t5.start()


nPhase2 = 0
def PortOutTh2():
    global fRunThread, nPhase2, TimeStep2, LoopsToGo2, DegPerStep
    while (fRunThread == 1):
        if(LoopsToGo2 < 0): #Negative value moves south
            if(nPhase2 == 0):
                GPIO.output(12, 1)
                GPIO.output(16, 0)
                GPIO.output(20, 0)
                GPIO.output(21, 0)
                nPhase2 = 1
            elif(nPhase2 == 1):
                GPIO.output(16, 1)
                GPIO.output(20, 0)
                GPIO.output(21, 0)
                GPIO.output(12, 0)
                nPhase2 = 2
            elif(nPhase2 == 2):
                GPIO.output(20, 1)
                GPIO.output(21, 0)
                GPIO.output(12, 0)
                GPIO.output(16, 0)
                nPhase2 = 3
            elif(nPhase2 == 3):
                GPIO.output(21, 1)
                GPIO.output(12, 0)
                GPIO.output(16, 0)
                GPIO.output(20, 0)
                nPhase2 = 0
            LoopsToGo2 += 1
#            print("LoopsToGo2 = %d" % LoopsToGo2)
        elif(LoopsToGo2 > 0):  # Positive value moves North
            if(nPhase2 == 0):
                GPIO.output(12, 1)
                GPIO.output(21, 0)
                GPIO.output(20, 0)
                GPIO.output(16, 0)
                nPhase2 = 3
            elif(nPhase2 == 3):
                GPIO.output(21, 1)
                GPIO.output(20, 0)
                GPIO.output(16, 0)
                GPIO.output(12, 0)
                nPhase2 = 2
            elif(nPhase2 == 2):
                GPIO.output(20, 1)
                GPIO.output(16, 0)
                GPIO.output(12, 0)
                GPIO.output(21, 0)
                nPhase2 = 1
            elif(nPhase2 == 1):
                GPIO.output(16, 1)
                GPIO.output(12, 0)
                GPIO.output(21, 0)
                GPIO.output(20, 0)
                nPhase2 = 0
            LoopsToGo2 -= 1
#            print("LoopsToGo2 = %d" % LoopsToGo2)
        else:
            GoN10Button.configure(bg=gray_default)
            GoN05Button.configure(bg=gray_default)
            GoN01Button.configure(bg=gray_default)
            GoS01Button.configure(bg=gray_default)
            GoS05Button.configure(bg=gray_default)
            GoS10Button.configure(bg=gray_default)
        time.sleep(TimeStep2)
t6 = threading.Thread(target=PortOutTh2)
t6.start()






#-----------------When Exiting
def on_closing():
#    camera.close()
    global fRunCamera, fRunImageUpdate, fRunDisplayUpdate
    fRunCamera=0
    fRunImageUpdate=0
    fRunDisplayUpdate=0
    t1.join()
    t2.join()
#    t3.join()
#    t4.join()
    global fRunThread
    fRunThread=0
    time.sleep(0.5)
    GPIO.cleanup()
    window.destroy()
    t5.join()
    t6.join()

window.protocol("WM_DELETE_WINDOW", on_closing)

window.mainloop()  #Starts GUI

