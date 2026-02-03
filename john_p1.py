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

# distance sensors
distance_sensor1 = robot.getDevice('D1')
distance_sensor1.enable(timestep)
distance_sensor2 = robot.getDevice('D2')
distance_sensor2.enable(timestep)
distance_sensor3 = robot.getDevice('D3')
distance_sensor3.enable(timestep)
distance_sensor4 = robot.getDevice('D4')
distance_sensor4.enable(timestep)
distance_sensor5 = robot.getDevice('D5')
distance_sensor5.enable(timestep)
distance_sensor6 = robot.getDevice('D6')
distance_sensor6.enable(timestep)
distance_sensor7 = robot.getDevice('D7')
distance_sensor7.enable(timestep)
distance_sensor8 = robot.getDevice('D8')
distance_sensor8.enable(timestep)

# GPS and orientation sensors
gps_sensor = robot.getDevice('gps')
gps_sensor.enable(timestep)

compass_sensor = robot.getDevice('inertial_unit')
compass_sensor.enable(timestep)

emitter = robot.getDevice('emitter')
emitter.setChannel(1)
emitter.send('john'.encode('utf-8'))

# globals for sensor data
prev_x, prev_z = 0, 0
x, y, z = 0, 0, 0
front_left, front_right, right_front, right_back, back_left, back_right, left_back, left_front = 0, 0, 0, 0, 0, 0, 0, 0
roll, pitch, yaw = 0, 0, 0
left_wheel_pos, right_wheel_pos = 0, 0

# 
time = 0
target_angle = 90
start_time = 0
stuck_time = 0
unstuck_time = 0

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
    global x, y, z, front_left, front_right, right_front, right_back, back_left, back_right, left_back, left_front
    global roll, pitch, yaw, left_wheel_pos, right_wheel_pos

    # GPS data
    gps_values = gps_sensor.getValues()
    x = gps_values[0] * 100 
    y = gps_values[1] * 100
    z = gps_values[2] * 100

    # intertial unit data
    compass_values = compass_sensor.getRollPitchYaw()
    roll = radToDegree(compass_values[0])
    pitch = radToDegree(compass_values[1])
    yaw = radToDegree(compass_values[2])

    # distance sensors
    front_left = distance_sensor1.getValue() * 320
    front_right = distance_sensor8.getValue() * 320
    right_front = distance_sensor7.getValue() * 320
    right_back = distance_sensor6.getValue() * 320
    back_left = distance_sensor3.getValue() * 320
    back_right = distance_sensor5.getValue() * 320
    left_back = distance_sensor4.getValue() * 320
    left_front = distance_sensor2.getValue() * 320

    # wheel position sensors
    left_wheel_pos = radToDegree(wheel_left_sensor.getValue())
    right_wheel_pos = radToDegree(wheel_right_sensor.getValue())

def printAllSensors():
    print('=== POSITION & ORIENTATION ===')
    print(f'Position - X: {x:.2f} cm, Y: {y:.2f} cm, Z: {z:.2f} cm')
    print(f'Orientation - Roll: {roll:.2f}°, Pitch: {pitch:.2f}°, Yaw: {yaw:.2f}°')
    
    print('\n=== DISTANCE SENSORS ===')
    print(f'Front left: {front_left:.2f}')
    print(f'Front right: {front_right:.2f}')
    print(f'Right front: {right_front:.2f}')
    print(f'Right back: {right_back:.2f}')
    print(f'Back left: {back_left:.2f}')
    print(f'Back right: {back_right:.2f}')
    print(f'Left back: {left_back:.2f}')
    print(f'Left front: {left_front:.2f}')
    
    print('\n=== WHEEL POSITIONS ===')
    print(f'Left wheel: {left_wheel_pos:.2f}°')
    print(f'Right wheel: {right_wheel_pos:.2f}°')
    print('-' * 50)

HOLE_THRESHOLD = 100
CLOSE_WALL = 20
WALL_THRESHOLD = 30 

def check_environment():
    dm = min(front_left, front_right)
    dr = right_front
    dl = left_front
    r45 = right_front
    l45 = left_front
    
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
    #     # print("hole detected")
    #     pass
    # elif hazard == "wall":
    #     #handle_hazard(hazard, location, value)
    #     # print("wall detected")
    #     pass
    
    wheel_left.setVelocity(0)
wheel_right.setVelocity(0)
