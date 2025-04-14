# Simple two DC motor robot class.  Exposes a simple LOGO turtle-like API for
# moving a robot forward, backward, and turning.  See RobotTest.py for an
# example of using this class.
# Author: Tony DiCola, Michael Schleider
# License: MIT License https://opensource.org/licenses/MIT
import time
import atexit

from Adafruit_MotorHAT import Adafruit_MotorHAT

class Motion:
    def __init__(self, addr=0x60, left_id=1, right_id=2, left_trim=0, right_trim=0,
                 stop_at_exit=True):
        self._mh = Adafruit_MotorHAT(addr)
        self._left = self._mh.getMotor(left_id)
        self._right = self._mh.getMotor(right_id)
        self._left_trim = left_trim
        self._right_trim = right_trim
        # Start with motors turned off.
        self._left.run(Adafruit_MotorHAT.RELEASE)
        self._right.run(Adafruit_MotorHAT.RELEASE)
        # Configure all motors to stop at program exit if desired.
        if stop_at_exit:
            atexit.register(self.stop)

    def _left_speed(self, speed):
        """Set the speed of the left motor, taking into account its trim offset.
        """
        assert 0 <= speed <= 255, 'Speed must be a value between 0 to 255 inclusive!'
        speed += self._left_trim
        speed = max(0, min(255, speed))  # Constrain speed to 0-255 after trimming.
        self._left.setSpeed(speed)

    def _right_speed(self, speed):
        """Set the speed of the right motor, taking into account its trim offset.
        """
        assert 0 <= speed <= 255, 'Speed must be a value between 0 to 255 inclusive!'
        speed += self._right_trim
        speed = max(0, min(255, speed))  # Constrain speed to 0-255 after trimming.
        self._right.setSpeed(speed)

    def stop(self):
        """Stop all movement."""
        self._left.run(Adafruit_MotorHAT.RELEASE)
        self._right.run(Adafruit_MotorHAT.RELEASE)

    def forward(self, speed, seconds=None):
        """Move forward at the specified speed (0-255).  Will start moving
        forward and return unless a seconds value is specified, in which
        case the robot will move forward for that amount of time and then stop.
        """
        # Set motor speed and move both forward.
        self._left_speed(speed)
        self._right_speed(speed)
        self._left.run(Adafruit_MotorHAT.FORWARD)
        self._right.run(Adafruit_MotorHAT.FORWARD)
        # If an amount of time is specified, move for that time and then stop.
        if seconds is not None:
            time.sleep(seconds)
            self.stop()

    def backward(self, speed, seconds=None):
        """Move backward at the specified speed (0-255).  Will start moving
        backward and return unless a seconds value is specified, in which
        case the robot will move backward for that amount of time and then stop.
        """
        # Set motor speed and move both backward.
        self._left_speed(speed)
        self._right_speed(speed)
        self._left.run(Adafruit_MotorHAT.BACKWARD)
        self._right.run(Adafruit_MotorHAT.BACKWARD)
        # If an amount of time is specified, move for that time and then stop.
        if seconds is not None:
            time.sleep(seconds)
            self.stop()

    def right(self, speed, seconds=None):
        """Spin to the right at the specified speed.  Will start spinning and
        return unless a seconds value is specified, in which case the robot will
        spin for that amount of time and then stop.
        """
        # Set motor speed and move both forward.
        self._left_speed(speed)
        self._right_speed(speed)
        self._left.run(Adafruit_MotorHAT.FORWARD)
        self._right.run(Adafruit_MotorHAT.BACKWARD)
        # If an amount of time is specified, move for that time and then stop.
        if seconds is not None:
            time.sleep(seconds)
            self.stop()

    def left(self, speed, seconds=None):
        """Spin to the left at the specified speed.  Will start spinning and
        return unless a seconds value is specified, in which case the robot will
        spin for that amount of time and then stop.
        """
        # Set motor speed and move both forward.
        self._left_speed(speed)
        self._right_speed(speed)
        self._left.run(Adafruit_MotorHAT.BACKWARD)
        self._right.run(Adafruit_MotorHAT.FORWARD)
        # If an amount of time is specified, move for that time and then stop.
        if seconds is not None:
            time.sleep(seconds)
            self.stop()