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
    DRIVE_CAL_SPEED = 200.0
    STEER_CAL_SPEED = 100.0

    def __init__(self):
        self._motor_drive = Motor(PORT_DRIVE, Direction.COUNTERCLOCKWISE, gears=[20, 28])
        self._motor_steering = Motor(PORT_STEERING, Direction.COUNTERCLOCKWISE)
        self._drive_dc = 0
        self._steer_angle = 0
        self._steer_angle_max = 0
    
    def calibrate_drive(self):
        print("drive: forward")
        self._motor_drive.run_time(speed=self.DRIVE_CAL_SPEED, time=2000.0, then=Stop.BRAKE)
        wait(500)
        
        print("drive: backward")
        self._motor_drive.run_time(speed=-self.DRIVE_CAL_SPEED, time=2000.0, then=Stop.BRAKE)
        wait(500)
    
    def calibrate_steering(self):
        print("steering: left")
        angle_left = self._motor_steering.run_until_stalled(
            speed=self.STEER_CAL_SPEED, then=Stop.HOLD, duty_limit=25
        )
        wait(500)
        
        print("steering: right")
        angle_right = self._motor_steering.run_until_stalled(
            speed=-self.STEER_CAL_SPEED, then=Stop.HOLD, duty_limit=25
        )
        wait(500)

        # We moved right to left and left > right because we moved
        # clockwise to go left, so choosing the center at d = (right - left) / 2
        # puts the center at 0 if we reset while at the left position
        d = (angle_right - angle_left) / 2.0
        angle_center = d
        self._steer_angle_max = d
        print(f"  right={angle_right}, left={angle_left}, center={angle_center}")
        
        print("steering: reset")
        self._motor_steering.reset_angle(angle_center)
        self._motor_steering.run_target(
            speed=self.STEER_CAL_SPEED, target_angle=0.0
        )
        wait(500)
    
    def process(self, buttons):
        # Drive
        if Button.RIGHT_MINUS in buttons:
            self._drive_dc -= 10
            print(f"drive: {self._drive_dc}")
            self._motor_drive.dc(self._drive_dc)  # bounded to -100%
        elif Button.RIGHT_PLUS in buttons:
            self._drive_dc += 10
            print(f"drive: {self._drive_dc}")
            self._motor_drive.dc(self._drive_dc)  # bounded to 100%
        elif Button.RIGHT in buttons:
            self._drive_dc = 0
            print("drive: brake")
            self._motor_drive.brake()

        # Steering
        if Button.LEFT_MINUS in buttons:
            self._steer_angle -= self._steer_angle_max / 10
            print(f"steering: {self._steer_angle}")
        elif Button.LEFT_PLUS in buttons:
            self._steer_angle += self._steer_angle_max / 10
            print(f"steering: {self._steer_angle}")
        elif Button.LEFT in buttons:
            self._steer_angle = 0.0
            print(f"steering: {self._steer_angle}")
        
        self._motor_steering.run_target(
            speed=150.0, target_angle=self._steer_angle, then=Stop.HOLD, wait=False
        )
    

class FunctionControl:
    COUPLING_SPEED = 500.0
    AUX_SPEED = 300.0

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
        self._motor_coupling.run_target(
            speed=self.COUPLING_SPEED, target_angle=assumed_angle_open, then=Stop.HOLD
        )
        wait(500)

        # Now the coupling should be open
        self._angle_open = self._motor_coupling.angle()
        self._angle_closed = int(self._angle_open - 90 / ratio)
        print(f"  actual: closed={self._angle_closed}, open={self._angle_open}")

        print("coupling: close")
        self._motor_coupling.run_target(
            speed=self.COUPLING_SPEED, target_angle=self._angle_closed, then=Stop.HOLD
        )
        wait(500)

    def calibrate_aux(self):
        print("aux: clockwise")
        self._motor_aux.run_time(speed=self.AUX_SPEED, time=2000.0, then=Stop.BRAKE)
        wait(500)

        print("aux: counter-clockwise")
        self._motor_aux.run_time(speed=-self.AUX_SPEED, time=2000.0, then=Stop.BRAKE)
        wait(500)

    def process(self, buttons):
        # Coupling
        if buttons == (Button.RIGHT_MINUS, ):
            print("coupling: open")
            self._motor_coupling.run_target(
                speed=self.COUPLING_SPEED, target_angle=self._angle_open, then=Stop.HOLD
            )
        elif buttons == (Button.RIGHT_PLUS, ):
            print("coupling: closed")
            self._motor_coupling.run_target(
                speed=self.COUPLING_SPEED, target_angle=self._angle_closed, then=Stop.HOLD
            )
        
        # Auxiliary
        if buttons == (Button.LEFT_MINUS, ):
            print("auxiliary: negative speed")
            self._motor_aux.run(-self.AUX_SPEED)
        elif buttons == (Button.LEFT_PLUS, ):
            print("auxiliary: positive speed")
            self._motor_aux.run(self.AUX_SPEED)
        else:
            self._motor_aux.hold()
    

# Connect remote
try:
    remote = Remote(name=REMOTE_ID)
    print(f"remote connected: {remote.name()}")
except OSError:
    print("failed to connect remote")
    halt()

remote.light.off()
for _ in range(5):
    remote.light.on(Color.ORANGE)
    wait(200)
    remote.light.off()
    wait(200)


# Instantiate controllers
drive_control = DriveControl()
function_control = FunctionControl()

# Calibration
calibrations = [
    ("steering", drive_control.calibrate_steering, Color.YELLOW),
    ("drive", drive_control.calibrate_drive, Color.BLUE),
    ("coupling", function_control.calibrate_coupling, Color.WHITE),
    ("auxiliary", function_control.calibrate_aux, Color.ORANGE)
]

try:
    print("wait for calibration...")
    while calibrations:
        while True:
            buttons = remote.buttons.pressed()
            if buttons == (Button.LEFT, Button.RIGHT):
                print("abort")
                hub.system.shutdown()
            elif Button.CENTER in buttons:
                break  # go to next calibration

            wait(LOOP_DELAY)

        name, calibration_fcn, color = calibrations.pop(0)
        print(f"calibrating: {name}")
        calibrate(calibration_fcn, color)
        print("done")
except Exception as e:
    print("error while calibrating")
    print(e)
    halt()


# Event loop
current_mode = MODE_INVALID
new_mode = MODE_IDLE
shutdown_count = 0
try:
    print("start eventloop...")
    while True:
        if current_mode != new_mode:
            print(f"{current_mode} -> {new_mode}")
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
            if current_mode == MODE_DRIVE:
                drive_control.process(buttons)
            elif current_mode == MODE_FUNCTION:
                function_control.process(buttons)
            else:
                pass  # idle

        wait(LOOP_DELAY)
except Exception as e:
    print("error in eventloop")
    print(e)
    halt()
