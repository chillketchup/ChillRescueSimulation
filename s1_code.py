import sys
import time
sys.path.append("C:\\Program Files\\Webots\\lib\\controller\\python")
from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

PI = 3.14159265

wheel_left = robot.getDevice("wheel1 motor")
wheel_right = robot.getDevice("wheel2 motor")

wheel_left.setPosition(float("Inf"))
wheel_right.setPosition(float("Inf"))

def set_wheel_velocities(v_left, v_right):
    v_left = min(v_left, 100)
    v_right = max(v_right, -100)

    v_left = min(v_left, 100)
    v_right = max(v_right, -100)

    wheel_left.setVelocity(v_left / 100 * 6.28)
    wheel_right.setVelocity(v_right / 100 * 6.28)

TURN90 = 0.325
MOVETILE = 0.925

sequence = [
    (100, 100, MOVETILE),
    (100, -100, TURN90), # left
    (100, 100, MOVETILE),
    (-100, 100, TURN90), # right
    (100, 100, MOVETILE * 2),
    (-100, 100, TURN90), #right 
    (100, 100, MOVETILE * 2.25),
    (-100, 100, TURN90), # right
    (100, 100, MOVETILE * 2.5),
    (100, -100, TURN90), # left 
    (100, 100, MOVETILE * 0.5),
    (-100, 100, TURN90), # right
    (100, 100, MOVETILE * 0.75),
]

current_step = 0
start_time = 0

while robot.step(timestep) != -1:
    
    if current_step < len(sequence):
        v_left, v_right, duration = sequence[current_step]
        
        if start_time == 0:
            set_wheel_velocities(v_left, v_right)
            start_time = robot.getTime()
        
        if robot.getTime() - start_time >= duration:
            current_step += 1
            start_time = 0
    else:
        wheel_left.setVelocity(0)
        wheel_right.setVelocity(0)
