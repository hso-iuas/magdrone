import rclpy
from rclpy.node import Node

from time import sleep
from std_msgs.msg import String
from std_msgs.msg import Empty
from custom_services.srv import DriveMagnet
from enum import Enum

import RPi.GPIO as GPIO

class MagnetDir(Enum):
    off = 0
    on = 1
    down = 2
    up = 3

# PIN DEFINITIONS

# Microstepping pins (DRV8825: M0, M1, M2)
M0 = 14  # Microstepping control pin M0
M1 = 15  # Microstepping control pin M1
M2 = 18  # Microstepping control pin M2

# Motor control pins
DIR = 20  # Direction control pin
STEP = 21  # Step signal pin
SLEEP_RESET = 17  # Tied together SLEEP + RESET pin

# Direction constants
CW = 1  # Clockwise rotation
CCW = 0  # Counter-clockwise rotation

# Motor specs
SPR = 200  # Steps per revolution (for full steps)

# Revolutions from attached to, fully retracted.
REVS = 40

# Define the function to rotate the motor
def step_motor(direction, steps, delay):
    GPIO.output(SLEEP_RESET, GPIO.HIGH)  # Enable the driver by bringing SLEEP/RESET high
    sleep(0.01)  # Wait a short time to allow driver to stabilize

    GPIO.output(DIR, direction)  # Set the desired direction (CW or CCW)

    # Generate pulses to rotate the motor
    for _ in range(steps):  # Repeat for the specified number of steps
        GPIO.output(STEP, GPIO.HIGH)  # Send a HIGH pulse
        sleep(delay)  # Wait a short time
        GPIO.output(STEP, GPIO.LOW)  # Send a LOW pulse (complete one step)
        sleep(delay)  # Wait before next step

    GPIO.output(SLEEP_RESET, GPIO.LOW)  # After stepping, disable driver to reduce heat

def setup_gpio():
    # GPIO SETUP

    GPIO.setmode(GPIO.BCM)  # Use BCM pin numbering

    # Set all motor-related pins as outputs
    GPIO.setup(DIR, GPIO.OUT)
    GPIO.setup(STEP, GPIO.OUT)
    GPIO.setup(SLEEP_RESET, GPIO.OUT)
    GPIO.setup(M0, GPIO.OUT)
    GPIO.setup(M1, GPIO.OUT)
    GPIO.setup(M2, GPIO.OUT)

    # MICROSTEPPING SETUP
    GPIO.output(M0, GPIO.HIGH)
    GPIO.output(M1, GPIO.LOW)
    GPIO.output(M2, GPIO.LOW)


class MagnetNode(Node):

    def __init__(self):
        super().__init__('magnet_node')

        self.srv = self.create_service(DriveMagnet, 'magdrone/magnet/drive', self.drive_magnet_callback)
        self.current_magnet_pos = REVS
        self.is_moving = False

    async def drive_magnet_callback(self, request, response):
        self.get_logger().info('Request received')

        # pos reached
        if self.current_magnet_pos == REVS and (request.direction == MagnetDir.up.value or request.direction == MagnetDir.on.value)\
           or self.current_magnet_pos == 0 and (request.direction == MagnetDir.down.value or request.direction == MagnetDir.off.value):
            self.get_logger().info('ALREADY IN POS')
            response.magnet_state = self.current_magnet_pos
            response.success = True
            return response

        # move
        try:
            self.is_moving = True
            await self.drive_magnet(request.direction)

            response.success = True
            self.get_logger().info('DONE MOVING')

        except Exception as e:
            self.get_logger().error(f'Failed to drive magnet: {e}')
            response.success = False

        finally:
            self.is_moving = False

        response.magnet_state = self.current_magnet_pos
        return response

    async def drive_magnet(self, direction):
        # MAIN CONTROL BLOCK
        try:
            if direction == MagnetDir.on.value:
                rev_amount = REVS - self.current_magnet_pos
                step_motor(CCW, SPR * rev_amount, 0.0005)  # 200 steps/rev × 40 revs = 10,000 steps
                self.current_magnet_pos = REVS


            elif direction == MagnetDir.off.value:
                step_motor(CW, SPR * self.current_magnet_pos, 0.0005)  # 0.0005 sec. delay between every step
                self.current_magnet_pos = 0

            elif direction == MagnetDir.down.value:
                self.current_magnet_pos -= 1
                step_motor(CW, SPR, 0.0005)  # 200 steps/rev × 40 revs = 10,000 steps

            elif direction == MagnetDir.up.value:
                self.current_magnet_pos += 1
                step_motor(CCW, SPR, 0.0005)  # 0.0005 sec. delay between every step

        # Allow the program to exit cleanly if user presses Ctrl+C
        except KeyboardInterrupt:
            print("Program interrupted by user")
            GPIO.cleanup()

def main(args=None):
    setup_gpio()
    rclpy.init(args=args)

    magnet_node = MagnetNode()

    rclpy.spin(magnet_node)

    magnet_node.destroy_node()
    rclpy.shutdown()
    GPIO.cleanup()


if __name__ == '__main__':
    main()
