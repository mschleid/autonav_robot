from jetbot import Robot
import time

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
    
if __name__ == "__main__":
    move(0.15, 0.0) # move forward
    time.sleep(0.5)
    move(0.35, 0.0) # move forward
    time.sleep(0.5)
    move(0.5, 0.0) # move forward
    time.sleep(0.5)
    move(0.7, 0.0) # move forward
    time.sleep(4)
    move(0.5, 0.0) # move forward
    time.sleep(0.5)
    move(0.35, 0.0) # move forward
    time.sleep(0.5)


    # time.sleep(7)
    # turn_right()
    # move(0.7,0.0)
    # time.sleep(4)
    stop()