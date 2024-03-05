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
print(f"Hub: {hub.system.name()}")
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


def calibrate(motor, color=Color.WHITE):
    hub.light.blink(color, [200, 300])
    remote.light.on(color)
    # m.calibrate()
    wait(5000)
    hub.light.off()
    remote.light.off()

halt()

# Controllers
class DriveControl:
    def __init__(self):
        pass


class FunctionControl:
    pass

# Connect remote
try:
    remote = Remote(name=REMOTE_ID)
    print(f"Remote connected: {remote.name()}")
except OSError:
    halt()

remote.light.off()

# Calibration
calibrations = [
    ("drive", None, Color.BLUE),
    ("steering", None, Color.YELLOW),
    ("coupling", None, Color.WHITE),
    ("auxiliary", None, Color.ORANGE)
]
while calibrations:
    while True:
        buttons = remote.buttons.pressed()
        if Button.CENTER in buttons:
            break
        wait(LOOP_DELAY)

    s, m, c = calibrations.pop()
    print(f"calibrating: {s}")
    calibrate(m, c)


# Event loop
current_mode = MODE_INVALID
new_mode = MODE_IDLE
shutdown_count = 0
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
        pass  # parsing of buttons is done later

    if current_mode == MODE_IDLE:
        pass
    elif current_mode == MODE_DRIVE:
        pass
    elif current_mode == MODE_FUNCTION:
        pass
    elif current_mode == MODE_SHUTDOWN:
        shutdown_count += 1
        if shutdown_count == SHUTDOWN_LIMIT:
            hub.system.shutdown()
    else:
        halt()

    wait(LOOP_DELAY)




# Driving
motor_drive = Motor(
    port=PORT_DRIVE,
    positive_direction=Direction.COUNTERCLOCKWISE,
    gears=[20, 28],
    reset_angle=False
)

print("DRIVE: forward")
#motor_drive.run_time(speed=100.0, time=2000.0, then=Stop.COAST)
wait(500)
print("DRIVE: backward")
#motor_drive.run_time(speed=-100.0, time=2000.0, then=Stop.COAST)
wait(500)

# Steering
motor_steering = Motor(
    port=PORT_STEERING,
    positive_direction=Direction.CLOCKWISE,
    reset_angle=False
)

print("STEER: right")
#angle_right = motor_steering.run_until_stalled(speed=100.0, then=Stop.BRAKE, duty_limit=25)
#print(f"angle_right={angle_right}")
wait(500)
print("STEER: left")
#angle_left = motor_steering.run_until_stalled(speed=-100.0, then=Stop.BRAKE, duty_limit=25)
#print(f"angle_right={angle_left}")
wait(500)

print("STEER: reset")
#angle_center = (angle_left - angle_right) / 2.0  # must be CW - CCW!
#print(f"center={angle_center}")
#motor_steering.reset_angle(angle_center)
print("STEER: center")
#motor_steering.run_target(speed=100.0, target_angle=0.0)
wait(500)

# Coupling
motor_coupling = Motor(
    port=PORT_COUPLING,
    positive_direction=Direction.COUNTERCLOCKWISE,
    gears=[12, 24],
    reset_angle=False
)

# ratio motor->coupler = 12/12 * 12/24 * 16/16 * 1/8
angle_close = motor_coupling.angle()
angle_open = int(angle_close + 90 / (12 / 24 * (1 / 8)))
print("COUPLING: open")
print(f"angle_open={angle_open}")
#motor_coupling.run_target(speed=200.0, target_angle=angle_open, then=Stop.HOLD)
wait(500)
print("COUPLING: close")
#motor_coupling.run_target(speed=200.0, target_angle=angle_close, then=Stop.HOLD)
print(f"angle_close={angle_close}")
wait(500)

# Auxiliary
motor_aux = Motor(
    port=PORT_AUX,
    positive_direction=Direction.CLOCKWISE,
    reset_angle=False
)

print("AUX: clockwise")
#motor_aux.run_time(speed=300.0, time=2000.0, then=Stop.COAST)
wait(500)
print("AUX: counter-clockwise")
#motor_aux.run_time(speed=-300.0, time=2000.0, then=Stop.COAST)
wait(500)

print("STOP")