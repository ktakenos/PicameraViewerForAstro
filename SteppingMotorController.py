# SteppingMotorController.py
# ==============================================================================
# Raspberry Pi 4 / Pi 5 両対応の步进モーター制御クラス.
#
# GPIO バックエンド自動検知 (RPi.GPIO インターフェースと完全互換):
#   - 1st:  rpi-lgpio  (pip install rpi-lgpio)
#            libgpiod ベースの RPi.GPIO ドロップイン置換. Pi4/Pi5 両対応.
#   - 2nd:  RPi.GPIO (wiringPi ベース)
#            Pi4/Bookworm以前で動作. Pi5(Bcm2712) では利用不可だが
#            念のためフォールバックとして残しておく.
#   - 3rd:  GPIO アダプタ (gpiozero/libgpiod 直接アクセス)
#            Raspberry Pi OS Bookworm に標準バンドル済みの gpiozero の
#            libgpiod バックエンドをラップして RPi.GPIO-like インターフェースを
#            提供する最終手段. OPi.GPIO は不要のため削除した.
#
# インターフェースは SteppingMotorControllerGPIO と完全一致である.
# MainApp.py はインポート先を変更することで自動的に新しいモジュールを使用できる.
# ==============================================================================


import time
from CameraController import CameraController


# ---------------------------------------------------------------------------
# GPIO バックエンド管理 (ラズパイバージョン自動検知 + ライブラリ切替)
# ---------------------------------------------------------------------------

_GPIO_MODULE = None     # 初期化済み GPIO モジュール (Lazy init)
_BACKEND_NAME = ''      # 使用中のバックエンド名 (ログ用)


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
    return 'BCM2712' in chip


# ---------------------------------------------------------------------------
# Fallback: gpiozero + libgpiod をラップした RPi.GPIO-like アダプタ
# ---------------------------------------------------------------------------

class _GPIOPiZeroAdapter:
    """gpiozero (libgpiod バックエンド) 上で RPi.GPIO のサブセットをエミュレート.
    
    本コードが必要とする機能は限定的なので以下のメソッドのみ実装する:
      - setmode(mode)   : BCM/OUT 定数のみ受け付ける (nop)
      - setup(pin, out) : OutputDevice を作成して内部辞書に格納
      - output(pin, val): on()/off() を呼び出す
      - cleanup()       : すべての OutputDevice.close()
    """
    
    BCM = 10
    OUT = 1
    
    def __init__(self):
        self._devices = {}
        try:
            from gpiozero import OutputDevice
            self._OutputDevice = OutputDevice
            from gpiozero.pins.lgpio import LGPIOFactory
            self._pin_factory = LGPIOFactory()
        except ImportError:
            try:
                from gpiozero import OutputDevice
                self._OutputDevice = OutputDevice
                self._pin_factory = None
            except ImportError:
                raise ImportError('gpiozero is not installed.')

    def setmode(self, mode):
        pass  # nop
    
    def setup(self, pin, mode):
        if self._pin_factory is not None:
            dev = self._OutputDevice(pin, pin_factory=self._pin_factory)
        else:
            dev = self._OutputDevice(pin)
        self._devices[pin] = dev

    def output(self, pin, value):
        dev = self._devices.get(pin)
        if dev is not None:
            if value:
                dev.on()
            else:
                dev.off()

    def cleanup(self):
        for dev in self._devices.values():
            try:
                dev.close()
            except Exception:
                pass
        self._devices.clear()


# ---------------------------------------------------------------------------
# GPIO バックエンド初期化 (優先度順で試行)
# ---------------------------------------------------------------------------

def _init_gpio():
    """利用可能な GPIO モジュールを優先度順に見つけて返す.
    
    Returns:
        tuple(module, name): GPIO モジュールとバックエンド名.
    
    Raises:
        RuntimeError: 有効な GPIO モジュールが見つからない場合.
    """
    
    # --- 1st: rpi-lgpio (libgpiod ベースの RPi.GPIO 互換) ----------
    try:
        import RPi.GPIO as GPIO
        vs = getattr(GPIO, '__version__', '').lower()
        if 'lgpio' in vs or 'libgpiod' in vs:
            return GPIO, 'rpi-lgpio (libgpiod)'
        # Pi5 上で import が成功したら rpi-lgpio の可能性が高い
        if _is_raspberry_pi5():
            return GPIO, 'RPi.GPIO on Pi5 (likely rpi-lgpio)'
    except ImportError:
        pass
    except Exception:
        pass
    
    # --- 2nd: 従来の RPi.GPIO (wiringPi ベース、Pi4 で動作) --------
    try:
        import RPi.GPIO as GPIO
        return GPIO, 'RPi.GPIO (wiringPi)'
    except ImportError:
        pass
    except Exception:
        pass
    
    # --- 3rd: gpiozero アダプタ ------------------------------------
    try:
        adapter = _GPIOPiZeroAdapter()
        return adapter, 'gpiozero adapter (libgpiod via gpiozero)'
    except ImportError:
        pass
    except Exception as e:
        print(f'[SteppingMotorController] Warning: gpiozero adapter failed: {e}')
    
    raise RuntimeError(
        '[SteppingMotorController] No compatible GPIO module found.\n'
        '  Please install one of the following:\n'
        '    Pi 4/5 (recommended): pip install rpi-lgpio\n'
        '    Pi 4 only:            pip install RPi.GPIO\n'
        '    Bookworm bundled:     gpiozero (usually pre-installed)\n'
    )




def _get_gpio():
    """（Lazy）初期化済みの GPIO モジュールを取得."""
    global _GPIO_MODULE, _BACKEND_NAME
    if _GPIO_MODULE is None:
        _GPIO_MODULE, _BACKEND_NAME = _init_gpio()
        print(f'[SteppingMotorController] Using GPIO backend: {_BACKEND_NAME}')
    return _GPIO_MODULE


# ---------------------------------------------------------------------------
# SteppingMotorController（本体）- 89行後に追加
# ---------------------------------------------------------------------------

class SteppingMotorController:
    """步进モーター制御クラス (Pi 4 / Pi 5 両対応).

    インターフェースは SteppingMotorControllerGPIO と完全一致なので、MainApp.py
    はインポート先を以下の1行に変更すればそのまま動作する::

        from SteppingMotorController import SteppingMotorController
    """

    def __init__(self, camera_controller):
        _get_gpio()                       # ライブラリをロード (Lazy init)
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


    # --- GPIO セットアップ / クリーンアップ --------------------------------

    def setup_gpio(self):
        gpio = _get_gpio()
        gpio.setmode(gpio.BCM)
        for pin in [6, 13, 19, 26]:
            gpio.setup(pin, gpio.OUT)
        for pin in [12, 16, 20, 21]:
            gpio.setup(pin, gpio.OUT)

    def cleanup_gpio(self):
        _get_gpio().cleanup()

    # --- 方向 / 速度設定 --------------------------------------------------

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

    # --- ループ数設定（度単位）--------------------------------------------

    def SetLoops2(self, value):    # value in degrees
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

    # --- PWM / Step 出力スレッド関数（GPIO バックエンド経由）--------------

    def PortOutTh1(self, fForwardDirection1, TimeStep1):
        gpio = _get_gpio()
        while self.fRunThread == 1:
            if fForwardDirection1 == 1:          # Positive value goes west
                if self.nPhase1 == 0:
                    gpio.output(6, 1)
                    gpio.output(13, 0)
                    gpio.output(19, 0)
                    gpio.output(26, 0)
                    self.nPhase1 = 1
                elif self.nPhase1 == 1:
                    gpio.output(13, 1)
                    gpio.output(19, 0)
                    gpio.output(26, 0)
                    gpio.output(6, 0)
                    self.nPhase1 = 2
                elif self.nPhase1 == 2:
                    gpio.output(19, 1)
                    gpio.output(26, 0)
                    gpio.output(6, 0)
                    gpio.output(13, 0)
                    self.nPhase1 = 3
                elif self.nPhase1 == 3:
                    gpio.output(26, 1)
                    gpio.output(6, 0)
                    gpio.output(13, 0)
                    gpio.output(19, 0)
                    self.nPhase1 = 0
            elif fForwardDirection1 == -1:
                if self.nPhase1 == 0:
                    gpio.output(6, 1)
                    gpio.output(26, 0)
                    gpio.output(19, 0)
                    gpio.output(13, 0)
                    self.nPhase1 = 3
                elif self.nPhase1 == 3:
                    gpio.output(26, 1)
                    gpio.output(19, 0)
                    gpio.output(13, 0)
                    gpio.output(6, 0)
                    self.nPhase1 = 2
                elif self.nPhase1 == 2:
                    gpio.output(19, 1)
                    gpio.output(13, 0)
                    gpio.output(6, 0)
                    gpio.output(26, 0)
                    self.nPhase1 = 1
                elif self.nPhase1 == 1:
                    gpio.output(13, 1)
                    gpio.output(6, 0)
                    gpio.output(26, 0)
                    gpio.output(19, 0)
                    self.nPhase1 = 0
            time.sleep(TimeStep1)





    def PortOutTh2(self, LoopsToGo2, DegPerStep, TimeStep2):
        gpio = _get_gpio()
        while self.fRunThread == 1:
            if LoopsToGo2 < 0:          # Negative value moves south
                if self.nPhase2 == 0:
                    gpio.output(12, 1)
                    gpio.output(16, 0)
                    gpio.output(20, 0)
                    gpio.output(21, 0)
                    self.nPhase2 = 1
                elif self.nPhase2 == 1:
                    gpio.output(16, 1)
                    gpio.output(20, 0)
                    gpio.output(21, 0)
                    gpio.output(12, 0)
                    self.nPhase2 = 2
                elif self.nPhase2 == 2:
                    gpio.output(20, 1)
                    gpio.output(21, 0)
                    gpio.output(12, 0)
                    gpio.output(16, 0)
                    self.nPhase2 = 3
                elif self.nPhase2 == 3:
                    gpio.output(21, 1)
                    gpio.output(12, 0)
                    gpio.output(16, 0)
                    gpio.output(20, 0)
                    self.nPhase2 = 0
                LoopsToGo2 += 1
            elif LoopsToGo2 > 0:        # Positive value moves North
                if self.nPhase2 == 0:
                    gpio.output(12, 1)
                    gpio.output(21, 0)
                    gpio.output(20, 0)
                    gpio.output(16, 0)
                    self.nPhase2 = 3
                elif self.nPhase2 == 3:
                    gpio.output(21, 1)
                    gpio.output(20, 0)
                    gpio.output(16, 0)
                    gpio.output(12, 0)
                    self.nPhase2 = 2
                elif self.nPhase2 == 2:
                    gpio.output(20, 1)
                    gpio.output(16, 0)
                    gpio.output(12, 0)
                    gpio.output(21, 0)
                    self.nPhase2 = 1
                elif self.nPhase2 == 1:
                    gpio.output(16, 1)
                    gpio.output(12, 0)
                    gpio.output(21, 0)
                    gpio.output(20, 0)
                    self.nPhase2 = 0
                LoopsToGo2 -= 1
            time.sleep(TimeStep2)


