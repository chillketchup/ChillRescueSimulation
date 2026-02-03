import sys
sys.path.append("C:\\Program Files\\Webots\\lib\\controller\\python")
from controller import Robot

robot = Robot()
wheel1 = robot.getDevice("wheel1 motor")
wheel2 = robot.getDevice("wheel2 motor")
enc1 = robot.getDevice("wheel1 sensor")
enc2 = robot.getDevice("wheel2 sensor")
wheel1.setPosition(float("inf"))
wheel2.setPosition(float("inf"))
timeStep = 8

gps = robot.getDevice("gps")
emitter = robot.getDevice("emitter")
gyro = robot.getDevice("imu")
cs = robot.getDevice("colour_sensor")
lidar = robot.getDevice("lidar")
receiver = robot.getDevice("receiver")
camera1 = robot.getDevice("Rcam")
camera2 = robot.getDevice("Lcam")

DM = robot.getDevice("DM")
DR = robot.getDevice("DR")
DL = robot.getDevice("DL")
R45 = robot.getDevice("R45")
L45 = robot.getDevice("L45")

DM.enable(timeStep)
DR.enable(timeStep)
DL.enable(timeStep)
R45.enable(timeStep)
L45.enable(timeStep)

gps.enable(timeStep)
gyro.enable(timeStep)
cs.enable(timeStep)
enc1.enable(timeStep)
enc2.enable(timeStep)
lidar.enable(timeStep)
camera1.enable(timeStep)
camera2.enable(timeStep)
receiver.enable(timeStep)

HOLE_THRESHOLD = 0.02
WALL_THRESHOLD = 0.3
CLOSE_WALL = 0.15
TURN_SPEED = 3.0
FORWARD_SPEED = 4.0
BACKUP_SPEED = -2.0

hole_timer = 0
wall_timer = 0
HOLE_TIME = 20
WALL_TIME = 15

def print_sensors():
    dm = int(DM.getValue() * 1000)
    dr = int(DR.getValue() * 1000)
    dl = int(DL.getValue() * 1000)
    r45 = int(R45.getValue() * 1000)
    l45 = int(L45.getValue() * 1000)

    print("middle:", dm, "right:", dr, "left:", dl, "right45:", r45, "left45", l45)

def check_environment():
    dm = DM.getValue()
    dr = DR.getValue()
    dl = DL.getValue()
    r45 = R45.getValue()
    l45 = L45.getValue()
    
    if dm > HOLE_THRESHOLD:
        return "hole", "middle", dm
    if dl > HOLE_THRESHOLD:
        return "hole", "left", dl
    if dr > HOLE_THRESHOLD:
        return "hole", "right", dr
    if r45 > HOLE_THRESHOLD:
        return "hole", "right_diagonal", r45
    if l45 > HOLE_THRESHOLD:
        return "hole", "left_diagonal", l45
    
    if dm < CLOSE_WALL:
        return "wall", "front", dm
    if dl < CLOSE_WALL:
        return "wall", "left", dl
    if dr < CLOSE_WALL:
        return "wall", "right", dr
    if l45 < WALL_THRESHOLD:
        return "wall", "left_diagonal", l45
    if r45 < WALL_THRESHOLD:
        return "wall", "right_diagonal", r45
    
    return "clear", "none", 0

# def handle_hazard(hazard_type, location, value):
#     if hazard_type == "hole":
#         print(f"Hole: {location}")
#         if location == "middle":
#             wheel1.setVelocity(BACKUP_SPEED)
#             wheel2.setVelocity(BACKUP_SPEED)
#             robot.step(timeStep * 5)
#             wheel1.setVelocity(TURN_SPEED)
#             wheel2.setVelocity(-TURN_SPEED)
#         elif location == "left" or location == "left_diagonal":
#             wheel1.setVelocity(TURN_SPEED)
#             wheel2.setVelocity(TURN_SPEED * 0.3)
#         elif location == "right" or location == "right_diagonal":
#             wheel1.setVelocity(TURN_SPEED * 0.3)
#             wheel2.setVelocity(TURN_SPEED)
#     elif hazard_type == "wall":
#         print(f"Wall: {location} - {value:.3f}")
#         if location == "front":
#             dl_val = DL.getValue()
#             dr_val = DR.getValue()
#             wheel1.setVelocity(BACKUP_SPEED)
#             wheel2.setVelocity(BACKUP_SPEED)
#             robot.step(timeStep * 3)
#             if dl_val > dr_val:
#                 wheel1.setVelocity(-TURN_SPEED)
#                 wheel2.setVelocity(TURN_SPEED)
#             else:
#                 wheel1.setVelocity(TURN_SPEED)
#                 wheel2.setVelocity(-TURN_SPEED)
#         elif location == "left" or location == "left_diagonal":
#             wheel1.setVelocity(TURN_SPEED)
#             wheel2.setVelocity(TURN_SPEED * 0.5)
#         elif location == "right" or location == "right_diagonal":
#             wheel1.setVelocity(TURN_SPEED * 0.5)
#             wheel2.setVelocity(TURN_SPEED)

while robot.step(timeStep) != -1:
    if hole_timer > 0:
        hole_timer -= 1
    if wall_timer > 0:
        wall_timer -= 1
    
    hazard, location, value = check_environment()

    print_sensors()
    
    if hazard == "hole":
        # handle_hazard(hazard, location, value)
        # print("hole detected")
        pass
    elif hazard == "wall":
        #handle_hazard(hazard, location, value)
        # print("wall detected")
        pass
    
    wheel1.setVelocity(0)
    wheel2.setVelocity(0)