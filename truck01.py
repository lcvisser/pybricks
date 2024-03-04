from pybricks.hubs import TechnicHub
from pybricks.pupdevices import Motor, Remote
from pybricks.parameters import Button, Color, Direction, Port, Stop
from pybricks.tools import wait


PORT_DRIVE = Port.C
PORT_STEERING = Port.A
PORT_COUPLING = Port.B
PORT_AUX = Port.D
REMOTE_ID = "ludo-remote01"

hub = TechnicHub()
print(f"Hub connected: {hub.system.name()}")
remote = Remote(name=REMOTE_ID)
print(f"Remote connected: {remote.name()}")

LOOP_DELAY = 100  # ms
shutdown_count = 0
while True:
    buttons = remote.buttons.pressed()
    if buttons == (Button.LEFT, Button.RIGHT):
        shutdown_count += 1
        print(f"shutdown: {shutdown_count}")
        if shutdown_count == 20:
            hub.system.shutdown()
    else:
        shutdown_count = 0
    wait(100)

# Driving
motor_drive = Motor(
    port=PORT_DRIVE,
    positive_direction=Direction.COUNTERCLOCKWISE,
    gears=[20, 28],
    reset_angle=False
)

print("DRIVE: forward")
motor_drive.run_time(speed=100.0, time=2000.0, then=Stop.COAST)
wait(500)
print("DRIVE: backward")
motor_drive.run_time(speed=-100.0, time=2000.0, then=Stop.COAST)
wait(500)

# Steering
motor_steering = Motor(
    port=PORT_STEERING,
    positive_direction=Direction.CLOCKWISE,
    reset_angle=False
)

print("STEER: right")
angle_right = motor_steering.run_until_stalled(speed=100.0, then=Stop.BRAKE, duty_limit=25)
print(f"angle_right={angle_right}")
wait(500)
print("STEER: left")
angle_left = motor_steering.run_until_stalled(speed=-100.0, then=Stop.BRAKE, duty_limit=25)
print(f"angle_right={angle_left}")
wait(500)

print("STEER: reset")
angle_center = (angle_left - angle_right) / 2.0  # must be CW - CCW!
print(f"center={angle_center}")
motor_steering.reset_angle(angle_center)
print("STEER: center")
motor_steering.run_target(speed=100.0, target_angle=0.0)
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
motor_coupling.run_target(speed=200.0, target_angle=angle_open, then=Stop.HOLD)
wait(500)
print("COUPLING: close")
motor_coupling.run_target(speed=200.0, target_angle=angle_close, then=Stop.HOLD)
print(f"angle_close={angle_close}")
wait(500)

# Auxiliary
motor_aux = Motor(
    port=PORT_AUX,
    positive_direction=Direction.CLOCKWISE,
    reset_angle=False
)

print("AUX: clockwise")
motor_aux.run_time(speed=300.0, time=2000.0, then=Stop.COAST)
wait(500)
print("AUX: counter-clockwise")
motor_aux.run_time(speed=-300.0, time=2000.0, then=Stop.COAST)
wait(500)

print("STOP")
