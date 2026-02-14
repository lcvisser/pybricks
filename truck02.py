from pybricks.hubs import TechnicHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Button, Color, Direction, Port, Stop
from pybricks.tools import wait
from pybricks.iodevices import XboxController

# Constants
PORT_DRIVE = Port.A
PORT_STEERING = Port.C
PORT_COUPLING = Port.D
PORT_AUX = Port.B

LOW_BATTERY = 7.0  # V
LOOP_DELAY = 50  # ms

STEERING_SPEED = 170  
COUPLING_SPEED = 100
AUX_SPEED = 30

# Global variables
hub = TechnicHub()
remote = XboxController()

motor_drive = Motor(PORT_DRIVE, Direction.COUNTERCLOCKWISE)
motor_steering = Motor(PORT_STEERING, Direction.CLOCKWISE)
motor_coupling = Motor(PORT_COUPLING, Direction.COUNTERCLOCKWISE)
motor_aux = Motor(PORT_AUX)


# Utility functions
def halt():
    hub.light.blink(Color.RED, [100, 100, 100, 100, 100, 500])
    for _ in range(10):
        wait(LOOP_DELAY)


def calibrate_steering(motor, speed):
    hub.light.blink(Color.GREEN, [500, 500])

    angle_left = motor.run_until_stalled(speed, duty_limit=25)
    wait(500)
    
    angle_right = motor.run_until_stalled(-speed, duty_limit=25)
    wait(500)

    half_range = (angle_right - angle_left) / 2.0
    motor.reset_angle(half_range)
    motor.run_target(speed, target_angle=0.0)
    wait(500)

    return half_range


def calibrate_coupling(motor, speed):
    hub.light.blink(Color.GREEN, [500, 500])

    angle_open = motor.run_until_stalled(speed, duty_limit=20)
    wait(500)
    
    angle_closed = motor.run_until_stalled(-speed, duty_limit=20)
    wait(500)
    
    return angle_open, angle_closed


# Calibration
try:
    max_steer_angle = calibrate_steering(motor_steering, STEERING_SPEED)
    open_angle, closed_angle = calibrate_coupling(motor_coupling, COUPLING_SPEED)
except:
    halt()


# Main event loop
try:
    while True:
        if hub.battery.voltage() > LOW_BATTERY:
            hub.light.on(Color.GREEN)
        else:
            hub.light.blink(Color.YELLOW, [500, 500])

        steering, _ = remote.joystick_left()
        motor_steering.run_target(STEERING_SPEED, max_steer_angle * steering / 100.0)

        drive_fwd, drive_bck = remote.triggers()
        motor_drive.dc(drive_fwd - drive_bck)
        
        buttons = remote.buttons.pressed()
        if Button.A in buttons:
            motor_coupling.run_target(COUPLING_SPEED, open_angle)
        elif Button.B in buttons:
            motor_coupling.run_target(COUPLING_SPEED, closed_angle)
        elif Button.X in buttons:
            motor_aux.dc(AUX_SPEED)
        elif Button.Y in buttons:
            motor_aux.dc(-AUX_SPEED)
        else:
            motor_aux.dc(0)

        if Button.GUIDE in buttons:
            hub.system.shutdown()
except:
    halt()
