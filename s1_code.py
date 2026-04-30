import sys
import time
import math
sys.path.append("C:\\Program Files\\Webots\\lib\\controller\\python")
from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

PI = 3.14159265

wheel_left = robot.getDevice("wheel1 motor")
wheel_right = robot.getDevice("wheel2 motor")

wheel_left.setPosition(float("Inf"))
wheel_right.setPosition(float("Inf"))

lidar = robot.getDevice("lidar")
lidar.enable(timestep)

compass = robot.getDevice("inertial_unit")
compass.enable(timestep)

gps = robot.getDevice("gps")
gps.enable(timestep)

max_velocity = 100
x = 0
y = 0

dir_snap = [0, 90, -90, 45, -45, 135, -135, 180]

def yaw():
    imu = compass.getRollPitchYaw()
    return int(imu[2] * 180/PI)

def set_orientation(target_angle):
    while robot.step(timestep) != -1:
        error = yaw() - target_angle
            
        if error > 360:
            error -= 360
        elif error < 0:
            error += 360
        
        speed = max(error, 10)
        speed = min(speed, max_velocity)
        
        if abs(error) <= 1:
            set_wheel_velocities(0, 0)
            return
        
        set_wheel_velocities(-speed, speed)

def gps_x():
    g = gps.getValues()
    g[0] = g[0] * 1000
    return int(g[0])

def gps_y():
    g = gps.getValues()
    g[2] = g[2] * 1000
    return int(g[2])

def set_wheel_velocities(v_left, v_right):
    v_left = min(v_left, 100)
    v_right = max(v_right, -100)

    v_left = min(v_left, 100)
    v_right = max(v_right, -100)

    wheel_left.setVelocity(v_left / 100 * 6.28)
    wheel_right.setVelocity(v_right / 100 * 6.28)

def dir():
    if yaw() in range(-22, 22):
        dir_value = 1
    elif yaw() in range(22, 67):
        dir_value = 4
    elif yaw() in range(67, 112):
        dir_value = 2
    elif yaw() in range(112, 157):
        dir_value = 6
    elif yaw() in range(-157, -112):
        dir_value = 7
    elif yaw() in range(-112, -67):
        dir_value = 3
    elif yaw() in range(-67, -22):
        dir_value = 5
    else:
        dir_value = 8

    return dir_value

def dist(direction):
    rangeImage = lidar.getRangeImage()

    if direction == "front":
        return rangeImage[1024] * 1000
    elif direction == "left":
        return rangeImage[1438] * 1000
    elif direction == "right":
        return rangeImage[1152] * 1000
    elif direction == "back":
        return rangeImage[1310] * 1000
  
TURN_90 = 325
MOVETILE = 925
WALL_THRESHOLD = 5

def forward():
    x = gps_x()
    y = gps_y()

    while robot.step(timestep) != -1:
        set_wheel_velocities(100, 100)
        error_x = abs(gps_x() - x)
        error_y = abs(gps_y() - y)

        print_data()

        if(error_x >= 119 or error_y >= 119):
            break
    
    set_wheel_velocities(0, 0)

    if dir() == 1:
        y = y - 1
    elif dir() == 2:
        x = x + 1
    elif dir() == 3:
        x = x - 1
    elif dir() == 4:
        x = x + math.sqrt(2)
        y = y - math.sqrt(2)
    elif dir() == 5:
        x = x - math.sqrt(2)
        y = y - math.sqrt(2)
    elif dir() == 6:
        x = x + math.sqrt(2)
        y = y + math.sqrt(2)
    elif dir() == 7:
        x = x - math.sqrt(2)
        y = y + math.sqrt(2)
    else:
        y = y + 1

def turn(direction):
    current = dir_snap[dir()-1]

    if direction == "left":
        angle = current - 90
    elif direction == "left45":
        angle = current - 45
    elif direction == "right45":
        angle = current + 45
    elif direction == "right":
        angle = current + 90
    elif direction == "right135":
        angle = current + 135
    elif direction == "back":
        angle = current + 180
    else:
        angle = current - 135
    
    if angle > 180:
        angle -= 360
    elif angle < -180:
        angle += 360
    
    set_orientation(angle)

current_step = 0
start_time = 0

def print_data():
    print('=== POSITION & ORIENTATION ===')
    print(f"Position - X: {gps_x():.2f} cm, Y: {gps_y():.2f} cm")
    print(f"Orientation - Yaw: {yaw():.2f}°")
    print(f"Direction: {dir()}°")
    
    print('\n=== DISTANCE SENSORS ===')
    print(f"front: {dist('front'):.2f}")
    print(f"right: {dist('left'):.2f}")
    print(f"left: {dist('right'):.2f}")
    print(f"back: {dist('back'):.2f}")

while robot.step(timestep) != -1:
    
    if dist('front') > WALL_THRESHOLD:
        forward()
    else:
        left_distance = dist("left")
        right_distance = dist("right")
        
        if left_distance > right_distance:
            turn("left")
        else:
            turn("right")

    robot.step(10)
