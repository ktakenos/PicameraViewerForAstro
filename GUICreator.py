# GUICreator.py
import tkinter as tk
import cv2
import numpy as np
from PIL import Image, ImageTk
from GUIHandler import GUIHandler

class GUICreator:
    def __init__(self, window, camera_controller, stepping_motor_controller, globals):
        self.window = window
        self.camera = camera_controller

        self.globals = globals
        self.gui_handler = GUIHandler(camera_controller, stepping_motor_controller, globals)
                
        self.create_gui()

    def create_gui(self):
        # Set up GUI
        self.window.wm_title("PiCamera Viewer")
        self.window.configure(background=self.globals.gray_default)

        # Graphics window
        self.imageFrame = tk.Frame(self.window, bg=self.globals.gray_default)
        self.imageFrame.grid(row=0, rowspan=3, column=0, padx=2, pady=2)
        self.lmain = tk.Label(self.imageFrame, bg=self.globals.gray_default)
        self.lmain.grid(row=0, rowspan=10, column=1, columnspan=3)

        DisplayImage = cv2.resize((self.camera.FrameImage/16).clip(0,255).astype(np.uint8), (int(self.camera.SensorW/5),int(self.camera.SensorH/5)))
        RGBImage = cv2.cvtColor(DisplayImage.astype(np.uint8), cv2.COLOR_RGB2BGR)
        img = Image.fromarray(RGBImage)
        imgtk = ImageTk.PhotoImage(img)
        self.lmain.imgtk = imgtk
        self.lmain.configure(image=imgtk)

        # ZoomImage window
        self.zoomFrame = tk.Frame(self.window, width=self.camera.ZoomWindowHW*2, height=self.camera.ZoomWindowHW*2, bg=self.globals.gray_default)
        self.zoomFrame.grid(row=0, column=1, padx=2, pady=2)
        self.zoomImage = tk.Label(self.zoomFrame, bg=self.globals.gray_default)
        self.zoomImage.grid(row=0, column=0, columnspan=4)

        RGBImageZoom = cv2.cvtColor(self.camera.ZoomImage, cv2.COLOR_RGB2BGR)
        img2 = Image.fromarray(RGBImageZoom)
        imgtk2 = ImageTk.PhotoImage(image=img2)
        self.zoomImage.imgtk = imgtk2
        self.zoomImage.configure(image=imgtk2)

        # For Threshold
        self.ThresLabel = tk.Label(self.zoomFrame, text="Threshold", bg=self.globals.gray_default, fg='white')
        self.ThresLabel.grid(row=1, column=0, sticky='ew')
        self.ThresScale = tk.Scale(self.zoomFrame, orient='horizontal', command=self.change_threshold,
                                   from_=10, to=250, bg=self.globals.gray_default, fg='white',
                                   troughcolor=self.globals.gray_default, highlightbackground=self.globals.gray_default,
                                   length=100, width=20)
        self.ThresScale.set(self.camera.vThreshold) 
        self.ThresScale.grid(row=1, column=1, sticky='ew')

        self.ZAttenLabel = tk.Label(self.zoomFrame, text="Atten", bg=self.globals.gray_default, fg='white')
        self.ZAttenLabel.grid(row=1, column=2, sticky='ew')
        self.ZAttenScale = tk.Scale(self.zoomFrame, orient='horizontal', command=self.change_z_atten,
                                   from_=0, to=100, resolution=2, bg=self.globals.gray_default, fg='white',
                                   troughcolor=self.globals.gray_default, highlightbackground=self.globals.gray_default,
                                   length=100, width=20)
        self.ZAttenScale.set(50)
        self.ZAttenScale.grid(row=1, column=3, sticky='ew')

        self.ThresButton = tk.Button(self.zoomFrame, text="Threshold", command=self.threshold_toggle,
                                     bg=self.globals.gray_default, fg='white')
        self.ThresButton.grid(row=2, column=0, columnspan=1, sticky="ew")

        # For Tracking
        self.TrackButton = tk.Button(self.zoomFrame, text="Track", command=self.track_toggle,
                                     bg=self.globals.gray_default, fg='white')
        self.TrackButton.grid(row=2, column=1, columnspan=1, sticky="ew")

        # For Dark
        self.DarkButton = tk.Button(self.zoomFrame, text="Dark", command=self.dark_toggle,
                                   bg=self.globals.gray_default, fg='white')
        self.DarkButton.grid(row=2, column=3, columnspan=1, sticky="ew")

        # Control Frame
        self.sliderFrame = tk.Frame(self.window, width=400, height=600, bg=self.globals.gray_default)
        self.sliderFrame.grid(row=1, rowspan=3, column=1, padx=2, pady=2)

        # For Analog Gain
        self.AGainLabel = tk.Label(self.sliderFrame, text="Analog Gain", bg=self.globals.gray_default, fg='white')
        self.AGainLabel.grid(row=0, column=0, sticky='ew')
        self.AGainScale = tk.Scale(self.sliderFrame, orient='horizontal', command=self.change_a_gain,
                                   from_=1, to=16, bg=self.globals.gray_default, fg='white',
                                   troughcolor=self.globals.gray_default, highlightbackground=self.globals.gray_default,
                                   length=100, width=20)
        self.AGainScale.set(self.camera.AnalogGain)  # Camera インスタンスのプロパティを使用
        self.AGainScale.grid(row=0, column=1, sticky='ew')

        # For Max Stack Adjustment
        self.MStackLabel = tk.Label(self.sliderFrame, text="MaxStack", bg=self.globals.gray_default, fg='white')
        self.MStackLabel.grid(row=0, column=2, sticky='ew')
        self.MStackScale = tk.Scale(self.sliderFrame, orient='horizontal', command=self.change_m_stack,
                                   from_=16, to=128, bg=self.globals.gray_default, fg='white',
                                   troughcolor=self.globals.gray_default, highlightbackground=self.globals.gray_default,
                                   length=100, width=20)
        self.MStackScale.set(self.camera.MStack)  # Camera インスタンスのプロパティを使用
        self.MStackScale.grid(row=0, column=3, sticky='ew')

        # For Exposure
        PowerOf2 = -2
        self.ExposButton = tk.Button(self.sliderFrame, text="Capture", command=self.ExposToggle, bg=self.globals.gray_default, fg='white')
        self.ExposButton.grid(row=5, column=0, sticky="ew")
        self.ExposLabel = tk.Label(self.sliderFrame, text="Exposure[s]", bg=self.globals.gray_default, fg='white')
        self.ExposLabel.grid(row=5, column=1, sticky='ew')
        self.globals.ExpSecStr.set('%1.3f' % self.camera.ExpSec)
        self.ExpSecLabel = tk.Label(self.sliderFrame, textvariable=self.globals.ExpSecStr, bg=self.globals.gray_default, fg='white')
        self.ExpSecLabel.grid(row=5, column=2, sticky='ew')
        self.ExposScale = tk.Scale(self.sliderFrame, orient='horizontal', command=self.change_expos,
                                   from_=-8, to=5, bg=self.globals.gray_default, fg='white', resolution=0.5,
                                   troughcolor=self.globals.gray_default, highlightbackground=self.globals.gray_default,
                                   length=100, width=20)
        self.ExposScale.set(PowerOf2)
        self.ExposScale.grid(row=5, column=3, sticky='ew')

        self.globals.CounterStr.set('%s' % self.camera.StackCounter)
        self.StackLabel = tk.Label(self.sliderFrame, text="Stack", bg=self.globals.gray_default, fg='white')
        self.StackLabel.grid(row=7, column=0, sticky='ew')
        self.CountLabel = tk.Label(self.sliderFrame, textvariable=self.globals.CounterStr, bg=self.globals.gray_default, fg='white')
        self.CountLabel.grid(row=7, column=1, sticky='ew')

        # For Stack Save
        self.SaveButton = tk.Button(self.sliderFrame, text="Save", command=self.save_stack, bg=self.globals.gray_default, fg='white')
        self.SaveButton.grid(row=7, column=2, sticky="ew")

        # For Stack Reset
        self.ResetButton=tk.Button(self.sliderFrame, text="Reset", command =self.reset_stack, bg=self.globals.gray_default, fg='white')
        self.ResetButton.grid(row=7,column=3,sticky="ew")

        # For Stack Show
        self.StackShowButton = tk.Button(self.sliderFrame, text="Show Stack", command=self.stack_show_toggle,
                                         bg=self.globals.gray_default, fg='white')
        self.StackShowButton.grid(row=9, column=0, sticky="ew")

        # For Level Adjust
        self.LevelButton = tk.Button(self.sliderFrame, text="Level Adjust", command=self.level_toggle,
                                     bg=self.globals.gray_default, fg='white')
        self.LevelButton.grid(row=9, column=1, columnspan=2, sticky="ew")

        # For Reset Level
        self.ResetLevelButton = tk.Button(self.sliderFrame, text="Reset Level", command=self.reset_level,
                                          bg=self.globals.gray_default, fg='white')
        self.ResetLevelButton.grid(row=9, column=3, sticky="ew")

        # For Brightness
        self.BrightLabel = tk.Label(self.sliderFrame, text="Level", bg=self.globals.gray_default, fg='white')
        self.BrightLabel.grid(row=10, column=0, sticky='ew')
        self.BrightScale = tk.Scale(self.sliderFrame, orient='horizontal', command=self.change_brightness,
                                    from_=0, to=0.5, bg=self.globals.gray_default, fg='white',
                                    resolution=0.0005,
                                    troughcolor=self.globals.gray_default, highlightbackground=self.globals.gray_default,
                                    length=100, width=20)
        self.BrightScale.set(self.globals.Brightness) 
        self.BrightScale.grid(row=11, column=0, sticky='ew')

        # For Dark Color Balance
        self.RLevelLabel = tk.Label(self.sliderFrame, text="RED", bg=self.globals.gray_default, fg='white')
        self.RLevelLabel.grid(row=10, column=1, sticky='ew')
        self.RLevelScale = tk.Scale(self.sliderFrame, orient='horizontal', command=self.r_level,
                                    from_=0, to=0.2, bg=self.globals.gray_default, fg='white',
                                    resolution=0.0005,
                                    troughcolor=self.globals.gray_default, highlightbackground=self.globals.gray_default,
                                    length=100, width=20)
        self.RLevelScale.set(self.globals.LevelR) 
        self.RLevelScale.grid(row=11, column=1, sticky='ew')

        self.GLevelLabel = tk.Label(self.sliderFrame, text="GREEN", bg=self.globals.gray_default, fg='white')
        self.GLevelLabel.grid(row=10, column=2, sticky='ew')
        self.GLevelScale = tk.Scale(self.sliderFrame, orient='horizontal', command=self.g_level,
                                    from_=0, to=0.2, bg=self.globals.gray_default, fg='white',
                                    resolution=0.0005,
                                    troughcolor=self.globals.gray_default, highlightbackground=self.globals.gray_default,
                                    length=100, width=20)
        self.GLevelScale.set(self.globals.LevelG) 
        self.GLevelScale.grid(row=11, column=2, sticky='ew')

        self.BLevelLabel = tk.Label(self.sliderFrame, text="BLUE", bg=self.globals.gray_default, fg='white')
        self.BLevelLabel.grid(row=10, column=3, sticky='ew')
        self.BLevelScale = tk.Scale(self.sliderFrame, orient='horizontal', command=self.b_level,
                                    from_=0, to=0.2, bg=self.globals.gray_default, fg='white',
                                    resolution=0.0005,
                                    troughcolor=self.globals.gray_default, highlightbackground=self.globals.gray_default,
                                    length=100, width=20)
        self.BLevelScale.set(self.globals.LevelB) 
        self.BLevelScale.grid(row=11, column=3, sticky='ew')

        # For Contrast
        self.ContrLabel = tk.Label(self.sliderFrame, text="Contrast", bg=self.globals.gray_default, fg='white')
        self.ContrLabel.grid(row=12, column=0, sticky='ew')

        self.Slope0Scale = tk.Scale(self.sliderFrame, orient='horizontal', command=self.ChangeSlope0,
                            from_=0, to=1.0, bg=self.globals.gray_default, fg='white',
                            resolution=0.01,
                            troughcolor=self.globals.gray_default,highlightbackground=self.globals.gray_default,
                            length=100, width=20)

        self.Slope0Scale.set(0.5)
        self.Slope0Scale.grid(row=12,column=1,sticky='ew')

        self.Slope1Scale = tk.Scale(self.sliderFrame, orient='horizontal', command=self.ChangeSlope1,
                            from_=1, to=30, bg=self.globals.gray_default, fg='white',
                            resolution=0.5,
                            troughcolor=self.globals.gray_default,highlightbackground=self.globals.gray_default,
                            length=100, width=20)

        self.Slope1Scale.set(10.0)
        self.Slope1Scale.grid(row=12,column=2,sticky='ew')

        self.DZScale = tk.Scale(self.imageFrame, orient='vertical', command=self.change_disp_zoom2,
                                from_=1, to=5, bg=self.globals.gray_default, fg='white', resolution=1,
                                troughcolor=self.globals.gray_default, highlightbackground=self.globals.gray_default,
                                length=50, width=20)
        self.DZScale.set(self.globals.DispZoomFactor) 
        self.DZScale.grid(row=0, column=0, sticky='ns')

        self.SetTrackButton=tk.Button(self.imageFrame, text="Normal", command =self.SetTrack, bg=self.globals.gray_default, fg='white')
        self.SetTrackButton.grid(row=10,column=2,sticky="ew")
        self.SetTrackButton.configure(bg=self.globals.ColorBLUE2)

        self.SetBackwardButton=tk.Button(self.imageFrame, text="<<5x", command =self.SetBackward, bg=self.globals.gray_default, fg='white')
        self.SetBackwardButton.grid(row=10,column=1,sticky="ew")

        self.SetFastforwardButton=tk.Button(self.imageFrame, text="5x>>", command =self.SetFastforward, bg=self.globals.gray_default, fg='white')
        self.SetFastforwardButton.grid(row=10,column=3,sticky="ew")

        self.NorthLabel = tk.Label(self.imageFrame, text="N", bg=self.globals.gray_default, fg='white')
        self.NorthLabel.grid(row=1,column=0,sticky='ew')

        self.GoN10Button=tk.Button(self.imageFrame, text="1.0", command =self.GoNorth10, bg=self.globals.gray_default, fg='white')
        self.GoN10Button.grid(row=2,column=0,sticky="ew")

        self.GoN05Button=tk.Button(self.imageFrame, text="0.5", command =self.GoNorth05, bg=self.globals.gray_default, fg='white')
        self.GoN05Button.grid(row=3,column=0,sticky="ew")

        self.GoN01Button=tk.Button(self.imageFrame, text="0.1", command =self.GoNorth01, bg=self.globals.gray_default, fg='white')
        self.GoN01Button.grid(row=4,column=0,sticky="ew")

        self.StopNSButton=tk.Button(self.imageFrame, text="0", command =self.StopNS, bg=self.globals.gray_default, fg='white')
        self.StopNSButton.grid(row=5,column=0,sticky="ew")

        self.GoS01Button=tk.Button(self.imageFrame, text="0.1", command =self.GoSouth01, bg=self.globals.gray_default, fg='white')
        self.GoS01Button.grid(row=6,column=0,sticky="ew")
        self.GoS05Button=tk.Button(self.imageFrame, text="0.5", command =self.GoSouth05, bg=self.globals.gray_default, fg='white')
        self.GoS05Button.grid(row=7,column=0,sticky="ew")
        self.GoS10Button=tk.Button(self.imageFrame, text="1.0", command =self.GoSouth10, bg=self.globals.gray_default, fg='white')
        self.GoS10Button.grid(row=8,column=0,sticky="ew")
        self.SouthLabel = tk.Label(self.imageFrame, text="S", bg=self.globals.gray_default, fg='white')
        self.SouthLabel.grid(row=9,column=0,sticky='ew')

    def setup_zoom(self):
        self.lmain.bind('<ButtonPress-1>', lambda event: self.gui_handler.ZoomPosition(event, self.zoomImage))
        self.lmain.bind('<MouseWheel>', lambda event: self.gui_handler.ChangeDispZoom(event))
        
    def change_threshold(self, vThres):
        self.gui_handler.change_threshold(vThres)

    def change_z_atten(self, ZoomAttenuation):
        self.gui_handler.change_z_atten(ZoomAttenuation, self.zoomImage)

    def threshold_toggle(self):
        self.gui_handler.threshold_toggle(self.ThresButton)

    def track_toggle(self):
        self.gui_handler.track_toggle(self.TrackButton)

    def dark_toggle(self):
        self.gui_handler.dark_toggle(self.DarkButton)

    def change_a_gain(self, AGain):
        self.gui_handler.change_a_gain(AGain)

    def change_m_stack(self, MaxStack):
        self.gui_handler.change_m_stack(MaxStack)

    def ExposToggle(self):
        self.gui_handler.ExposToggle(self.ExposButton)

    def change_expos(self, PowOf2):
        self.gui_handler.change_expos(PowOf2, self.ExpSecLabel)

    def save_stack(self):
        self.gui_handler.save_stack(self.CountLabel)

    def reset_stack(self):
        self.gui_handler.reset_stack(self.CountLabel)

    def stack_show_toggle(self):
        self.gui_handler.stack_show_toggle(self.StackShowButton)

    def level_toggle(self):
        self.gui_handler.level_toggle(self.LevelButton)

    def reset_level(self):
        self.gui_handler.reset_level(self.BrightScale)

    def change_brightness(self, ScaleValue):
        self.gui_handler.change_brightness(ScaleValue)

    def r_level(self, Value):
        self.gui_handler.r_level(Value)

    def g_level(self, Value):
        self.gui_handler.g_level(Value)

    def b_level(self, Value):
        self.gui_handler.b_level(Value)

    def change_disp_zoom2(self, ScaleValue):
        self.gui_handler.change_disp_zoom2(ScaleValue, self.CountLabel, self.lmain, self.zoomImage)

    def ChangeSlope0(self, ScaleValue):
        self.gui_handler.ChangeSlope0(ScaleValue)

    def ChangeSlope1(self, ScaleValue):
        self.gui_handler.ChangeSlope1(ScaleValue)

    def SetTrack(self):
        self.gui_handler.SetTrack(self.SetTrackButton, self.SetBackwardButton, self.SetFastforwardButton)

    def SetBackward(self):
        self.gui_handler.SetBackward(self.SetTrackButton, self.SetBackwardButton, self.SetFastforwardButton)

    def SetFastforward(self):
        self.gui_handler.SetFastforward(self.SetTrackButton, self.SetBackwardButton, self.SetFastforwardButton)

    def SetLoops2(self, value):   #value in degrees
        self.gui_handler.SetLoops2(value)

    def GoNorth10(self):
        self.gui_handler.GoNorth10(self.GoN10Button)

    def GoNorth05(self):
        self.gui_handler.GoNorth05(self.GoN05Button)

    def GoNorth01(self):
        self.gui_handler.GoNorth01(self.GoN01Button)

    def StopNS(self):
        self.StopNS(self.GoN10Button, self.GoN05Button, self.GoN01Button, self.GoS10Button, self.GoS05Button, self.GoS01Button)

    def GoSouth01(self):
        self.gui_handler.GoSouth01(self.GoS01Button)

    def GoSouth05(self):
        self.gui_handler.GoSouth05(self.GoS05Button)

    def GoSouth10(self):
        self.gui_handler.GoSouth10(self.GoS10Button)
