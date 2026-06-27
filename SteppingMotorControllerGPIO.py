import RPi.GPIO as GPIO
import time
from CameraController import CameraController

class SteppingMotorController:
    def __init__(self, camera_controller):
        self.setup_gpio()
        self.nPhase1 = 0
        self.nPhase2 = 0
        self.fRunThread = 1

        # Ratio inside stepper
        self.StepPerRev = 32
        self.RatioInGear = 64
        self.RatioOutGear = 4
        self.DegWarm = 4
        self.DegPerStep = self.DegWarm / (self.RatioOutGear * self.RatioInGear * self.StepPerRev)
        self.DegPerSec = 360 / (24 * 60 * 60)
        self.GuideStep = self.DegPerStep / self.DegPerSec

        self.fForwardDirection1 = 1
        self.TimeStep1 = self.GuideStep
        self.LoopsToGo2 = 0
        self.TimeStep2 = 0.02
        self.Camera = CameraController(camera_controller, globals)

    def setup_gpio(self):
        GPIO.setmode(GPIO.BCM)
        for pin in [6, 13, 19, 26]:
            GPIO.setup(pin, GPIO.OUT)
        for pin in [12, 16, 20, 21]:
            GPIO.setup(pin, GPIO.OUT)

    def cleanup_gpio(self):
        GPIO.cleanup()

    def SetTrack(self):
        if self.Camera.fTrack == 1:
            return
        self.fForwardDirection1 = 1
        self.TimeStep1 = self.GuideStep

    def SetBackward(self):
        if self.Camera.fTrack == 1:
            return
        self.fForwardDirection1 = -1
        self.TimeStep1 = self.GuideStep / 4.0

    def SetFastforward(self):
        if self.Camera.fTrack == 1:
            return
        self.fForwardDirection1 = 1
        self.TimeStep1 = self.GuideStep / 5.0

    def SetLoops2(self, value):  # value in degrees
        self.LoopsToGo2 = int(float(value) / self.DegPerStep)

    def GoNorth10(self):
        if self.LoopsToGo2 == 0:
            self.SetLoops2(1.0)

    def GoNorth05(self):
        if self.LoopsToGo2 == 0:
            self.SetLoops2(0.5)

    def GoNorth01(self):
        if self.LoopsToGo2 == 0:
            self.SetLoops2(0.1)

    def StopNS(self):
        if self.LoopsToGo2 == 0:
            return
        else:
            self.SetLoops2(0)

    def GoSouth01(self):
        if self.LoopsToGo2 == 0:
            self.SetLoops2(-0.1)

    def GoSouth05(self):
        if self.LoopsToGo2 == 0:
            self.SetLoops2(-0.5)

    def GoSouth10(self):
        if self.LoopsToGo2 == 0:
            self.SetLoops2(-1.0)

    def PortOutTh1(self, fForwardDirection1, TimeStep1):
        while self.fRunThread == 1:
            if fForwardDirection1 == 1:  # Positive value goes west
                if self.nPhase1 == 0:
                    GPIO.output(6, 1)
                    GPIO.output(13, 0)
                    GPIO.output(19, 0)
                    GPIO.output(26, 0)
                    self.nPhase1 = 1
                elif self.nPhase1 == 1:
                    GPIO.output(13, 1)
                    GPIO.output(19, 0)
                    GPIO.output(26, 0)
                    GPIO.output(6, 0)
                    self.nPhase1 = 2
                elif self.nPhase1 == 2:
                    GPIO.output(19, 1)
                    GPIO.output(26, 0)
                    GPIO.output(6, 0)
                    GPIO.output(13, 0)
                    self.nPhase1 = 3
                elif self.nPhase1 == 3:
                    GPIO.output(26, 1)
                    GPIO.output(6, 0)
                    GPIO.output(13, 0)
                    GPIO.output(19, 0)
                    self.nPhase1 = 0
            elif fForwardDirection1 == -1:
                if self.nPhase1 == 0:
                    GPIO.output(6, 1)
                    GPIO.output(26, 0)
                    GPIO.output(19, 0)
                    GPIO.output(13, 0)
                    self.nPhase1 = 3
                elif self.nPhase1 == 3:
                    GPIO.output(26, 1)
                    GPIO.output(19, 0)
                    GPIO.output(13, 0)
                    GPIO.output(6, 0)
                    self.nPhase1 = 2
                elif self.nPhase1 == 2:
                    GPIO.output(19, 1)
                    GPIO.output(13, 0)
                    GPIO.output(6, 0)
                    GPIO.output(26, 0)
                    self.nPhase1 = 1
                elif self.nPhase1 == 1:
                    GPIO.output(13, 1)
                    GPIO.output(6, 0)
                    GPIO.output(26, 0)
                    GPIO.output(19, 0)
                    self.nPhase1 = 0
            time.sleep(TimeStep1)

    def PortOutTh2(self, LoopsToGo2, DegPerStep, TimeStep2):
        while self.fRunThread == 1:
            if LoopsToGo2 < 0:  # Negative value moves south
                if self.nPhase2 == 0:
                    GPIO.output(12, 1)
                    GPIO.output(16, 0)
                    GPIO.output(20, 0)
                    GPIO.output(21, 0)
                    self.nPhase2 = 1
                elif self.nPhase2 == 1:
                    GPIO.output(16, 1)
                    GPIO.output(20, 0)
                    GPIO.output(21, 0)
                    GPIO.output(12, 0)
                    self.nPhase2 = 2
                elif self.nPhase2 == 2:
                    GPIO.output(20, 1)
                    GPIO.output(21, 0)
                    GPIO.output(12, 0)
                    GPIO.output(16, 0)
                    self.nPhase2 = 3
                elif self.nPhase2 == 3:
                    GPIO.output(21, 1)
                    GPIO.output(12, 0)
                    GPIO.output(16, 0)
                    GPIO.output(20, 0)
                    self.nPhase2 = 0
                LoopsToGo2 += 1
            elif LoopsToGo2 > 0:  # Positive value moves North
                if self.nPhase2 == 0:
                    GPIO.output(12, 1)
                    GPIO.output(21, 0)
                    GPIO.output(20, 0)
                    GPIO.output(16, 0)
                    self.nPhase2 = 3
                elif self.nPhase2 == 3:
                    GPIO.output(21, 1)
                    GPIO.output(20, 0)
                    GPIO.output(16, 0)
                    GPIO.output(12, 0)
                    self.nPhase2 = 2
                elif self.nPhase2 == 2:
                    GPIO.output(20, 1)
                    GPIO.output(16, 0)
                    GPIO.output(12, 0)
                    GPIO.output(21, 0)
                    self.nPhase2 = 1
                elif self.nPhase2 == 1:
                    GPIO.output(16, 1)
                    GPIO.output(12, 0)
                    GPIO.output(21, 0)
                    GPIO.output(20, 0)
                    self.nPhase2 = 0
                LoopsToGo2 -= 1
            time.sleep(TimeStep2)