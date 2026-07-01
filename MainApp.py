import tkinter as tk
import threading
import time
from GUICreator import GUICreator
from GUIHandler import GUIHandler
from CameraController import CameraController
from SteppingMotorControllerGPIO import SteppingMotorController

class GlobalVariables:
    def __init__(self):
        self.fThreshold = 0
        self.Brightness = 0.1
        self.Contrast = 0.0
        self.LevelR = 0.11
        self.LevelG = 0.1
        self.LevelB = 0.114
        self.Center = float(0.25)
        self.Slope0 = float(0.5)
        self.Slope1 = float(10.0)
        self.DispZoomFactor = 1
        self.DispScale = 32
        self.fForwardDirection1 = 1
        self.fRunThread = 1
        self.LoopsToGo2 = 0
        self.TimeStep2 = 0.02
        self.fRunDisplayUpdate = False
        self.fRunZoomUpdate = False
        self.fStackShow = 0
        self.fLevel = 0
        self.gray_default = "#222222"
        self.ColorRED = '#A00000'
        self.ColorGREEN = '#007000'
        self.ColorBLUE = '#0000A0'
        self.ColorBLUE2 = '#303080'
        self.ExpSecStr = tk.StringVar()
        self.CounterStr=tk.StringVar()
        
class MainApp:
    def __init__(self, root):
        self.root = root
        self.globals = GlobalVariables()
        self.stepping_motor_controller = SteppingMotorController(self.globals)
        self.camera_controller = CameraController(self.stepping_motor_controller, self.globals)
        self.gui_creator = GUICreator(root, self.camera_controller, self.stepping_motor_controller, self.globals)
        self.gui_handler = GUIHandler(self.camera_controller, self.stepping_motor_controller, self.globals)
        
        self.root.after(100, self.gui_creator.setup_zoom)

        self.setup_app()

    def setup_app(self):
        self.camera_controller.reset_camera()
        self.camera_controller.start_run_thread()
        self.camera_controller.start_convert_thread()

        self.DisplayUpdateThread = threading.Thread(target=self.run_display_update)
        self.DisplayUpdateThread.start()

        self.stepping_motor_controller.setup_gpio()

        self.MotorAscThread = threading.Thread(target=self.stepping_motor_controller.PortOutTh1, args=(self.globals.fForwardDirection1, self.globals.TimeStep2))
        self.MotorDecThread = threading.Thread(target=self.stepping_motor_controller.PortOutTh2, args=(self.globals.LoopsToGo2, 0.1, self.globals.TimeStep2))  # Assuming DegPerStep is 0.1
        self.MotorAscThread.start()
        self.MotorDecThread.start()

    def run_display_update(self):
        while True:
            self.camera_controller.CaptureFrames()
            if self.globals.fRunDisplayUpdate:
                self.gui_handler.update_display(self.gui_creator.CountLabel, self.gui_creator.lmain, self.gui_creator.TrackButton)
                if self.camera_controller.fTrack == 1:
                    if self.camera_controller.fLost == 1:
                        self.gui_creator.TrackButton.configure(bg=self.globals.ColorRED)
                    else:
                        self.gui_creator.TrackButton.configure(bg=self.globals.ColorBLUE2)
                self.globals.fRunDisplayUpdate = 0
            if self.globals.fRunZoomUpdate:
                self.gui_handler.update_zoom(self.gui_creator.zoomImage)
                self.globals.fRunZoomUpdate = 0
            time.sleep(0.2)

    def stop_threads(self):
        self.globals.fRunDisplayUpdate = False
        self.globals.fRunZoomUpdate = False
        self.stepping_motor_controller.fRunThread = 0

    def on_closing(self):
        self.stop_threads()
        print('Camera is closing')
        
        # Stop camera threads first
        self.camera_controller.stop_run_thread()
        self.camera_controller.stop_convert_thread()

        print('Application is closing')
        # Stop display update thread
        self.DisplayUpdateThread.join(timeout=5)

        # Stop motor threads
        print('GPIO Thread is closing')
        self.MotorAscThread.join(timeout=5)
        self.MotorDecThread.join(timeout=5)

        # Cleanup GPIO
        self.stepping_motor_controller.cleanup_gpio()

        # Destroy the main window
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = MainApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()