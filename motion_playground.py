# HOW THIS WORKS:
# 1. run ipython3
# 2. within ipython3, enter: run motion_playground.py
# - within ipython3, use move(linear, angular) to move the robot
# - within ipython3, use stop() to stop the robot

from jetbot import Robot
robot = Robot()

def clamp(value: float, min_value: float = -1.0, max_value: float = 1.0) -> float:
    return max(min_value, min(max_value, value))

def move(linear: float, angular: float) -> None:
    global robot
    left = clamp(linear - angular)
    right = clamp(linear + angular)

    robot.set_motors(left, right)

def stop() -> None:
    global robot
    robot.set_motors(0, 0)

    