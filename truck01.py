from pybricks.hubs import TechnicHub
from pybricks.pupdevices import Motor, Remote
from pybricks.parameters import Button, Color, Direction, Port, Stop
from pybricks.tools import wait


PORT_DRIVE = Port.C
PORT_STEERING = Port.A
PORT_COUPLING = Port.B
PORT_AUX = Port.D
REMOTE_ID = "ludo-remote01"

MODE_INVALID = ""
MODE_IDLE = "idle"
MODE_DRIVE = "drive"
MODE_FUNCTION = "function"
MODE_SHUTDOWN = "shutdown"

MODES_COLORS = {
    MODE_IDLE: Color.GREEN,
    MODE_DRIVE: Color.BLUE,
    MODE_FUNCTION: Color.WHITE,
    MODE_SHUTDOWN: Color.RED,
}

LOOP_DELAY = 100  # ms
SHUTDOWN_LIMIT = 20  # x LOOP_DELAY ms

hub = TechnicHub()
print(f"hub: {hub.system.name()}")
hub.light.animate([v for _, v in MODES_COLORS.items()], interval=500)

remote = None


# Utility functions
def halt():
    print("halted")

    hub.light.blink(Color.RED, [100, 100, 100, 100, 100, 500])
    s = False
    while True:
        if remote:
            remote.light.on(Color.RED) if s else remote.light.off()
            s = not s

        wait(LOOP_DELAY)


def toggle_mode(current_mode, modes):
    try:
        idx = modes.index(current_mode)
    except ValueError:
        idx = 0

    return modes[(idx + 1) % len(modes)]


def calibrate(calibration_fcn, color=Color.WHITE):
    hub.light.blink(color, [200, 300])
    remote.light.on(color)
    calibration_fcn()
    hub.light.off()
    remote.light.off()


# Controllers
class DriveControl:
    def __init__(self):
        self._motor_drive = Motor(PORT_DRIVE, Direction.COUNTERCLOCKWISE, gears=[20, 28])
        self._motor_steering = Motor(PORT_STEERING, Direction.CLOCKWISE)
    
    def calibrate_drive(self):
        print("drive: forward")
        self._motor_drive.run_time(speed=100.0, time=2000.0, then=Stop.BRAKE)
        wait(500)
        
        print("drive: backward")
        self._motor_drive.run_time(speed=-100.0, time=2000.0, then=Stop.BRAKE)
        wait(500)
    
    def calibrate_steering(self):
        print("steering: right")
        angle_right = self._motor_steering.run_until_stalled(speed=100.0, then=Stop.HOLD, duty_limit=25)
        wait(500)
        
        print("steering: left")
        angle_left = self._motor_steering.run_until_stalled(speed=-100.0, then=Stop.HOLD, duty_limit=25)
        wait(500)

        # We moved right to left and left < right because we moved
        # counter-clockwise to go left, so choosing the center at
        # d = (left - right) / 2 puts the center at 0 if we reset while at the
        # left position
        angle_center = (angle_left - angle_right) / 2.0
        print(f"  right={angle_right}, left={angle_left}, center={angle_center}")
        
        print("steering: reset")
        self._motor_steering.reset_angle(angle_center)
        self._motor_steering.run_target(speed=100.0, target_angle=0.0)
        wait(500)
    
    def process(self, mode, buttons):
        pass
    

class FunctionControl:
    def __init__(self):
        self._motor_coupling = Motor(PORT_COUPLING, Direction.COUNTERCLOCKWISE)
        self._motor_aux = Motor(PORT_AUX, Direction.CLOCKWISE)

    def calibrate_coupling(self):
        # Ratio motor->coupler = 12/12 * 12/24 * 16/16 * 1/8 = 8/8
        ratio = (12 / 24) * (1 / 8)

        # Assume the coupling was closed, then opening is 90 degrees further; to
        # calibrate we overshoot to make sure we reach the limits (if it was
        # actually open already, it'll slip)
        assumed_angle_closed = self._motor_coupling.angle()
        assumed_angle_open = int(assumed_angle_closed + 100 / ratio)
        print(f"  assume: closed={assumed_angle_closed}, open={assumed_angle_open}")
        
        print("coupling: open")
        self._motor_coupling.run_target(speed=200.0, target_angle=assumed_angle_open, then=Stop.HOLD)
        wait(500)

        # Now the coupling should be open
        self._angle_open = self._motor_coupling.angle()
        self._angle_closed = int(self._angle_open - 90 / ratio)
        print(f"  actual: closed={self._angle_closed}, open={self._angle_open}")

        print("coupling: close")
        self.motor_coupling.run_target(speed=200.0, target_angle=self._angle_closed, then=Stop.HOLD)
        wait(500)

    def calibrate_aux(self):
        print("aux: clockwise")
        self._motor_aux.run_time(speed=300.0, time=2000.0, then=Stop.BRAKE)
        wait(500)

        print("aux: counter-clockwise")
        self._motor_aux.run_time(speed=-300.0, time=2000.0, then=Stop.BRAKE)
        wait(500)

    def process(self, mode, buttons):
        pass
    

# Connect remote
try:
    remote = Remote(name=REMOTE_ID)
    print(f"remote connected: {remote.name()}")
except OSError:
    halt()

remote.light.off()
for _ in range(10):
    remote.light.on(Color.ORANGE)
    wait(200)
    remote.light.off()
    wait(200)


# Instantiate controllers
drive_control = DriveControl()
function_control = FunctionControl()

# Calibration
calibrations = [
    ("drive", drive_control.calibrate_drive, Color.BLUE),
    ("steering", drive_control.calibrate_steering, Color.YELLOW),
    ("coupling", function_control.calibrate_coupling, Color.WHITE),
    ("auxiliary", function_control.calibrate_aux, Color.ORANGE)
]

try:
    while calibrations:
        while True:
            buttons = remote.buttons.pressed()
            if Button.CENTER in buttons:
                break
            wait(LOOP_DELAY)

        name, calibration_fcn, color = calibrations.pop()
        print(f"calibrating: {name}")
        calibrate(calibration_fcn, color)
except OSError as e:
    print("error while calibrating")
    print(e)
    halt()


# Event loop
current_mode = MODE_INVALID
new_mode = MODE_IDLE
shutdown_count = 0
try:
    while True:
        if current_mode != new_mode:
            current_mode = new_mode
            c = MODES_COLORS[current_mode]
            remote.light.on(c)
            hub.light.on(c)
        
        buttons = remote.buttons.pressed()
        if buttons == (Button.LEFT, Button.RIGHT):
            new_mode = MODE_SHUTDOWN
        elif buttons == (Button.CENTER, ):
            new_mode = toggle_mode(current_mode, [MODE_IDLE, MODE_DRIVE, MODE_FUNCTION])
        else:
            pass  # parsing of buttons is done by controllers

        if current_mode == MODE_SHUTDOWN:
            shutdown_count += 1
            if shutdown_count == SHUTDOWN_LIMIT:
                hub.system.shutdown()
        else:
            shutdown_count = 0

        wait(LOOP_DELAY)
except OSError as e:
    print("error in eventloop")
    print(e)
    halt()
