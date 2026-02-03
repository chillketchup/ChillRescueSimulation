import sys
import math
sys.path.append("C:\\Program Files\\Webots\\lib\\controller\\python")
from controller import Robot

PI = 3.14159265

class RobotController:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        self._initialize_motors()
        self._initialize_sensors()
        
        self.position = {'x': 0, 'y': 0, 'z': 0}
        self.orientation = {'roll': 0, 'pitch': 0, 'yaw': 0}
        self.distances = {
            'dm': 0,  # middle
            'dr': 0,  # right
            'dl': 0,  # left
            'r45': 0,  # right diagonal
            'l45': 0   # left diagonal
        }
        self.wheel_positions = {'left': 0, 'right': 0}
        
    def _initialize_motors(self):
        self.wheel_left = self.robot.getDevice('wheel1 motor')
        self.wheel_left.setPosition(float('inf'))
        self.wheel_right = self.robot.getDevice('wheel2 motor')
        self.wheel_right.setPosition(float('inf'))
    
    def _initialize_sensors(self):
        self.dm_sensor = self.robot.getDevice('DM')
        self.dm_sensor.enable(self.timestep)
        
        self.dr_sensor = self.robot.getDevice('DR')
        self.dr_sensor.enable(self.timestep)
        
        self.dl_sensor = self.robot.getDevice('DL')
        self.dl_sensor.enable(self.timestep)
        
        self.r45_sensor = self.robot.getDevice('R45')
        self.r45_sensor.enable(self.timestep)
        
        self.l45_sensor = self.robot.getDevice('L45')
        self.l45_sensor.enable(self.timestep)
    
        self.wheel_left_sensor = self.robot.getDevice('wheel1 sensor')
        self.wheel_left_sensor.enable(self.timestep)
        
        self.wheel_right_sensor = self.robot.getDevice('wheel2 sensor')
        self.wheel_right_sensor.enable(self.timestep)
    
        self.gps_sensor = self.robot.getDevice('gps')
        self.gps_sensor.enable(self.timestep)
        
        self.compass_sensor = self.robot.getDevice('imu')
        self.compass_sensor.enable(self.timestep)
    
    def set_wheel_velocities(self, left_velocity, right_velocity):
        left_velocity = left_velocity / 10 * 6.28
        right_velocity = right_velocity / 10 * 6.28
        
        self.wheel_left.setVelocity(left_velocity)
        self.wheel_right.setVelocity(right_velocity)
    
    def stop_motors(self):
        self.wheel_left.setVelocity(0)
        self.wheel_right.setVelocity(0)
    
    def read_all_sensors(self):
        gps_values = self.gps_sensor.getValues()
        self.position['x'] = gps_values[0] * 100
        self.position['y'] = gps_values[1] * 100
        self.position['z'] = gps_values[2] * 100
        
        compass_values = self.compass_sensor.getRollPitchYaw()
        self.orientation['roll'] = math.degrees(compass_values[0])
        self.orientation['pitch'] = math.degrees(compass_values[1])
        self.orientation['yaw'] = math.degrees(compass_values[2])
        
        self.distances['dm'] = self.dm_sensor.getValue() * 320
        self.distances['dr'] = self.dr_sensor.getValue() * 320
        self.distances['dl'] = self.dl_sensor.getValue() * 320
        self.distances['r45'] = self.r45_sensor.getValue() * 320
        self.distances['l45'] = self.l45_sensor.getValue() * 320
        
        self.wheel_positions['left'] = math.degrees(self.wheel_left_sensor.getValue())
        self.wheel_positions['right'] = math.degrees(self.wheel_right_sensor.getValue())
    
    def print_sensor_data(self):
        print('=== POSITION & ORIENTATION ===')
        print(f"Position - X: {self.position['x']:.2f} cm, Y: {self.position['y']:.2f} cm, Z: {self.position['z']:.2f} cm")
        print(f"Orientation - Roll: {self.orientation['roll']:.2f}°, Pitch: {self.orientation['pitch']:.2f}°, Yaw: {self.orientation['yaw']:.2f}°")
        
        print('\n=== DISTANCE SENSORS ===')
        print(f"DM (middle): {self.distances['dm']:.2f}")
        print(f"DR (right): {self.distances['dr']:.2f}")
        print(f"DL (left): {self.distances['dl']:.2f}")
        print(f"R45 (right diagonal): {self.distances['r45']:.2f}")
        print(f"L45 (left diagonal): {self.distances['l45']:.2f}")
        
        print('\n=== WHEEL POSITIONS ===')
        print(f"Left wheel: {self.wheel_positions['left']:.2f}°")
        print(f"Right wheel: {self.wheel_positions['right']:.2f}°")
        print('-' * 50)
    
    # def check_environment(self):
    #     dm = self.distances['dm']
    #     dl = self.distances['dl']
    #     dr = self.distances['dr']
    #     r45 = self.distances['r45']
    #     l45 = self.distances['l45']
        
    #     # Check for holes (high distance values)
    #     if dm > HOLE_THRESHOLD:
    #         return "hole", "middle", dm
    #     if dl > HOLE_THRESHOLD:
    #         return "hole", "left", dl
    #     if dr > HOLE_THRESHOLD:
    #         return "hole", "right", dr
    #     if r45 > HOLE_THRESHOLD:
    #         return "hole", "right_diagonal", r45
    #     if l45 > HOLE_THRESHOLD:
    #         return "hole", "left_diagonal", l45
        
    #     # Check for walls (low distance values)
    #     if dm < CLOSE_WALL:
    #         return "wall", "front", dm
    #     if dl < CLOSE_WALL:
    #         return "wall", "left", dl
    #     if dr < CLOSE_WALL:
    #         return "wall", "right", dr
    #     if l45 < WALL_THRESHOLD:
    #         return "wall", "left_diagonal", l45
    #     if r45 < WALL_THRESHOLD:
    #         return "wall", "right_diagonal", r45
        
    #     return "clear", "none", 0
    
    def run(self):
        """Main control loop"""
        while self.robot.step(self.timestep) != -1:
            self.read_all_sensors()
            self.print_sensor_data()
            
            # if hazard == "hole":
            #     self.handle_hole(location, value)
            # elif hazard == "wall":
            #     self.handle_wall(location, value)
            
            self.stop_motors()

controller = RobotController()
controller.run()
