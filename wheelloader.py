from pybricks.hubs import TechnicHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Button, Color, Direction, Port, Stop
from pybricks.tools import wait
from pybricks.iodevices import XboxController


# Constants
PORT_DRIVE = Port.B
PORT_STEERING = Port.D
PORT_ARM = Port.A
PORT_TILT = Port.C

LOW_BATTERY = 7.0  # V
LOOP_DELAY = 100  # ms

STEERING_SPEED = 150  
TILT_SPEED = 100  # % duty cyle

# Global variables
hub = TechnicHub()
remote = XboxController()

motor_drive = Motor(PORT_DRIVE, Direction.COUNTERCLOCKWISE)
motor_steering = Motor(PORT_STEERING)
motor_arm = Motor(PORT_ARM)
motor_tilt = Motor(PORT_TILT)


# Utility functions
def halt():
    hub.light.blink(Color.RED, [100, 100, 100, 100, 100, 500])
    for _ in range(10):
        wait(LOOP_DELAY)


def calibrate(motor, speed):
    hub.light.blink(Color.GREEN, [500, 500])

    angle_right = motor.run_until_stalled(speed, duty_limit=25)
    wait(500)
    
    angle_left = motor.run_until_stalled(-speed, duty_limit=25)
    wait(500)

    half_range = (angle_left - angle_right) / 2.0
    motor.reset_angle(half_range)
    motor.run_target(speed, target_angle=0.0)
    wait(500)

    return half_range


# Calibration
try:
    max_steer_angle = calibrate(motor_steering, STEERING_SPEED)
except:
    halt()


# Main event loop
try:
    hub.system.set_stop_button(Button.GUIDE)

    while True:
        if hub.battery.voltage() > LOW_BATTERY:
            hub.light.on(Color.GREEN)
        else:
            hub.light.blink(Color.YELLOW, [500, 500])

        steering, drive = remote.joystick_left()
        motor_drive.dc(drive)
        motor_steering.run_target(STEERING_SPEED, max_steer_angle * steering / 100.0)

        arm_down, arm_up = remote.triggers()
        motor_arm.dc(arm_up - arm_down)
        
        buttons = remote.buttons.pressed()
        if Button.A in buttons:
            motor_tilt.dc(-TILT_SPEED)  # down
        elif Button.B in buttons:
            motor_tilt.dc(TILT_SPEED)  # up
        else:
            motor_tilt.dc(0)

        if Button.GUIDE in buttons:
            hub.system.shutdown()
except:
    halt()
