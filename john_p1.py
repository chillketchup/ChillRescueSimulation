import sys
sys.path.append("C:\\Program Files\\Webots\\lib\\controller\\python")
from controller import Robot

PI = 3.14159265

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# motor setup
wheel_left = robot.getDevice('wheel1 motor')
wheel_left.setPosition(float('inf'))
wheel_right = robot.getDevice('wheel2 motor')
wheel_right.setPosition(float('inf'))

# wheel position sensors
wheel_left_sensor = robot.getDevice('wheel1 sensor')
wheel_left_sensor.enable(timestep)
wheel_right_sensor = robot.getDevice('wheel2 sensor')
wheel_right_sensor.enable(timestep)

# distance sensors - using your specific sensor names
DM = robot.getDevice('DM')
DM.enable(timestep)
DR = robot.getDevice('DR')
DR.enable(timestep)
DL = robot.getDevice('DL')
DL.enable(timestep)
R45 = robot.getDevice('R45')
R45.enable(timestep)
L45 = robot.getDevice('L45')
L45.enable(timestep)

# GPS and orientation sensors
gps_sensor = robot.getDevice('gps')
gps_sensor.enable(timestep)

compass_sensor = robot.getDevice('inertial_unit')
compass_sensor.enable(timestep)

emitter = robot.getDevice('emitter')
emitter.setChannel(1)
emitter.send('john'.encode('utf-8'))

# Define thresholds
HOLE_THRESHOLD = 100
CLOSE_WALL = 20
WALL_THRESHOLD = 30

# globals for sensor data
prev_x, prev_z = 0, 0
x, y, z = 0, 0, 0
roll, pitch, yaw = 0, 0, 0
left_wheel_pos, right_wheel_pos = 0, 0

# Sensor values (now using your specific sensors)
dm_value, dr_value, dl_value, r45_value, l45_value = 0, 0, 0, 0, 0

# Timers and state variables
time = 0
target_angle = 90
start_time = 0
stuck_time = 0
unstuck_time = 0
hole_timer = 0  # Added missing variable
wall_timer = 0  # Added missing variable

algorithm = "find_corner"
state = "turn"
state_next = "forward"
turn_dir = ""

def radToDegree(rad):
    return rad * 180 / PI

def setWheelVelocities(left_velocity, right_velocity):
    left_velocity = left_velocity / 10 * 6.28
    right_velocity = right_velocity / 10 * 6.28

    wheel_left.setVelocity(left_velocity)
    wheel_right.setVelocity(right_velocity)

def readAllSensors():
    global x, y, z, roll, pitch, yaw, left_wheel_pos, right_wheel_pos
    global dm_value, dr_value, dl_value, r45_value, l45_value

    # GPS data
    gps_values = gps_sensor.getValues()
    x = gps_values[0] * 100 
    y = gps_values[1] * 100
    z = gps_values[2] * 100

    # inertial unit data
    compass_values = compass_sensor.getRollPitchYaw()
    roll = radToDegree(compass_values[0])
    pitch = radToDegree(compass_values[1])
    yaw = radToDegree(compass_values[2])

    # distance sensors
    dm_value = DM.getValue() * 320 
    dr_value = DR.getValue() * 320
    dl_value = DL.getValue() * 320
    r45_value = R45.getValue() * 320
    l45_value = L45.getValue() * 320

    # wheel position sensors
    left_wheel_pos = radToDegree(wheel_left_sensor.getValue())
    right_wheel_pos = radToDegree(wheel_right_sensor.getValue())

def printAllSensors():
    print('=== POSITION & ORIENTATION ===')
    print(f'Position - X: {x:.2f} cm, Y: {y:.2f} cm, Z: {z:.2f} cm')
    print(f'Orientation - Roll: {roll:.2f}°, Pitch: {pitch:.2f}°, Yaw: {yaw:.2f}°')
    
    print('\n=== DISTANCE SENSORS ===')
    print(f'DM (middle): {dm_value:.2f}')
    print(f'DR (right): {dr_value:.2f}')
    print(f'DL (left): {dl_value:.2f}')
    print(f'R45 (right diagonal): {r45_value:.2f}')
    print(f'L45 (left diagonal): {l45_value:.2f}')
    
    print('\n=== WHEEL POSITIONS ===')
    print(f'Left wheel: {left_wheel_pos:.2f}°')
    print(f'Right wheel: {right_wheel_pos:.2f}°')
    print('-' * 50)

def check_environment():
    if dm_value > HOLE_THRESHOLD:
        return "hole", "middle", dm_value
    if dl_value > HOLE_THRESHOLD:
        return "hole", "left", dl_value
    if dr_value > HOLE_THRESHOLD:
        return "hole", "right", dr_value
    if r45_value > HOLE_THRESHOLD:
        return "hole", "right_diagonal", r45_value
    if l45_value > HOLE_THRESHOLD:
        return "hole", "left_diagonal", l45_value
    
    if dm_value < CLOSE_WALL:
        return "wall", "front", dm_value
    if dl_value < CLOSE_WALL:
        return "wall", "left", dl_value
    if dr_value < CLOSE_WALL:
        return "wall", "right", dr_value
    if l45_value < WALL_THRESHOLD:
        return "wall", "left_diagonal", l45_value
    if r45_value < WALL_THRESHOLD:
        return "wall", "right_diagonal", r45_value
    
    return "clear", "none", 0

# Main loop
while robot.step(timestep) != -1:
    readAllSensors()
    printAllSensors()

    if hole_timer > 0:
        hole_timer -= 1
    if wall_timer > 0:
        wall_timer -= 1
    
    hazard, location, value = check_environment()
    
    # if hazard == "hole":
    #     # handle_hazard(hazard, location, value)
    #     print(f"Hole detected: {location} with value {value}")
    # elif hazard == "wall":
    #     # handle_hazard(hazard, location, value)
    #     print(f"Wall detected: {location} with value {value}")
    
    wheel_left.setVelocity(0)
    wheel_right.setVelocity(0)
