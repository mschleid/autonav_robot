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

def ramp_up():
    for i in range(0, 81):
        move(i/100, 0)
        time.sleep(0.025)

def ramp_down():
    for i in range(0, 81):
        move((80-i)/100, 0)
        time.sleep(0.025)

def ramp_down_half():
    for i in range(0, 41):
        move((80-i)/100, 0)
        time.sleep(0.025)

def ramp_up_half():
    for i in range(40, 81):
        move(i/100, 0)
        time.sleep(0.025)

def turn_right():
    move(0.4, -0.3)
    time.sleep(0.55)
    
if __name__ == "__main__":
    ramp_up()
    time.sleep(1.75)
    ramp_down_half()
    turn_right()
    ramp_up_half()
    move(0.8,0)
    time.sleep(1.5)
    ramp_down()
    time.sleep(0.02)
    stop()



    # time.sleep(7)
    # turn_right()
    # move(0.7,0.0)
    # time.sleep(4)
    stop()