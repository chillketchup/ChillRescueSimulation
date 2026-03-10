import sys
import math
import numpy as np
import matplotlib.pyplot as plt
sys.path.append("C:\\Program Files\\Webots\\lib\\controller\\python")
from controller import Robot

PI = 3.14159265
TILE_SIZE = 11
ORIGIN_X = -66
ORIGIN_Z = 24

class LiDARProcessor:
    def __init__(self, robot, timestep, lidar_name='lidar'):
        self.robot = robot
        self.timestep = timestep
        
        self.lidar = self.robot.getDevice(lidar_name)
        self.lidar.enable(timestep)
        self.lidar.enablePointCloud()
        
        self.hit_positions = []
        self.timestamped_data = []
        
        self.horizontal_resolution = None
        self.num_layers = None
        self.min_range = None
        self.max_range = None
        
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        
    def get_lidar_params(self):
        if self.horizontal_resolution is None:
            self.horizontal_resolution = self.lidar.getHorizontalResolution()
            self.num_layers = self.lidar.getNumberOfLayers()
            self.min_range = self.lidar.getMinRange()
            self.max_range = self.lidar.getMaxRange()
        
        return {
            'horizontal_resolution': self.horizontal_resolution,
            'num_layers': self.num_layers,
            'min_range': self.min_range,
            'max_range': self.max_range,
            'fov': self.lidar.getFov(),
            'vertical_fov': self.lidar.getVerticalFov()
        }
    
    def get_hit_positions(self, robot_position, robot_orientation):
        range_image = self.lidar.getRangeImage()
        
        if not range_image:
            return []
        
        params = self.get_lidar_params()
        
        range_array = np.array(range_image)
        range_array = range_array.reshape((params['num_layers'], params['horizontal_resolution']))
        
        horizontal_angles = np.linspace(-params['fov']/2, params['fov']/2, params['horizontal_resolution'])
        vertical_angles = np.linspace(-params['vertical_fov']/2, params['vertical_fov']/2, params['num_layers'])
        
        self.hit_positions = []
        
        robot_yaw = math.radians(robot_orientation['yaw'])
        
        for i in range(params['num_layers']):
            for j in range(params['horizontal_resolution']):
                distance = range_array[i, j]
                
                if distance < self.min_range or distance > self.max_range:
                    continue
                
                distance_cm = distance * 100
                
                x_lidar = distance_cm * math.cos(vertical_angles[i]) * math.cos(horizontal_angles[j])
                y_lidar = distance_cm * math.cos(vertical_angles[i]) * math.sin(horizontal_angles[j])
                
                x_robot = x_lidar
                y_robot = -y_lidar
                
                cos_yaw = math.cos(robot_yaw)
                sin_yaw = math.sin(robot_yaw)
                
                x_world = robot_position['x'] + x_robot * cos_yaw - y_robot * sin_yaw
                z_world = robot_position['z'] + x_robot * sin_yaw + y_robot * cos_yaw
                
                self.hit_positions.append([x_world, z_world])
        
        timestamp = self.robot.getTime()
        self.timestamped_data.append({
            'timestamp': timestamp,
            'robot_position': robot_position.copy(),
            'robot_orientation': robot_orientation.copy(),
            'hit_positions': self.hit_positions.copy()
        })
        
        self.plot_top_down_view(robot_position)
        
        return self.hit_positions
    
    def plot_top_down_view(self, robot_position):
        self.ax.clear()
        
        if self.hit_positions:
            hit_array = np.array(self.hit_positions)
            self.ax.scatter(hit_array[:, 0], hit_array[:, 1], c='blue', s=2, alpha=0.6, label='LiDAR hits')
        
        self.ax.scatter(robot_position['x'], robot_position['z'], c='red', s=200, marker='^', label='Robot')
        
        arrow_length = 50
        dx = arrow_length * math.cos(math.radians(self.robot_orientation['yaw']))
        dy = arrow_length * math.sin(math.radians(self.robot_orientation['yaw']))
        self.ax.arrow(robot_position['x'], robot_position['z'], dx, dy, 
                     head_width=20, head_length=20, fc='red', ec='red', alpha=0.8)
        
        self.ax.set_xlabel('X (cm)')
        self.ax.set_ylabel('Z (cm)')
        self.ax.set_title('Top-Down LiDAR View')
        self.ax.grid(True, alpha=0.3)
        self.ax.legend()
        self.ax.axis('equal')
        
        if self.hit_positions:
            hit_array = np.array(self.hit_positions)
            margin = 100
            x_min = min(hit_array[:, 0].min(), robot_position['x']) - margin
            x_max = max(hit_array[:, 0].max(), robot_position['x']) + margin
            z_min = min(hit_array[:, 1].min(), robot_position['z']) - margin
            z_max = max(hit_array[:, 1].max(), robot_position['z']) + margin
            self.ax.set_xlim(x_min, x_max)
            self.ax.set_ylim(z_min, z_max)
        
        plt.draw()
        plt.pause(0.01)
    
    def clear_data(self):
        self.hit_positions = []
        self.timestamped_data = []

class RobotController:
    
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        self._initialize_motors()
        self._initialize_sensors()
        
        self.lidar = LiDARProcessor(self.robot, self.timestep, 'lidar')
        
        self.position = {'x': 0, 'y': 0, 'z': 0}
        self.orientation = {'roll': 0, 'pitch': 0, 'yaw': 0}
        self.distances = {
            'dm': 0,
            'dr': 0,
            'dl': 0,
            'r45': 0,
            'l45': 0
        }
        self.wheel_positions = {'left': 0, 'right': 0}
        self.current_tile = {'x': 0, 'z': 0}
        
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
    
    def calculate_current_tile(self):
        relative_x = abs(self.position['x'] - ORIGIN_X)
        relative_z = abs(self.position['z'] - ORIGIN_Z)
        
        tile_x = math.floor(relative_x / TILE_SIZE * 2) / 2 + 0.5
        tile_z = math.floor(relative_z / TILE_SIZE * 2) / 2 + 0.5
        
        return {'x': tile_x, 'z': tile_z}
    
    def calculate_target_angle_distance(self, target_x, target_z):
        difference_x = target_x - self.position['x']
        difference_z = target_z - self.position['z']

        angle = math.degrees(math.atan2(difference_z, difference_x))
        distance = math.sqrt(math.pow(difference_x, 2) + math.pow(difference_z, 2))

        return (angle, distance)
    
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

        self.current_tile = self.calculate_current_tile()
        
        compass_values = self.compass_sensor.getRollPitchYaw()
        self.orientation['roll'] = math.degrees(compass_values[0])
        self.orientation['pitch'] = math.degrees(compass_values[1])
        self.orientation['yaw'] = math.degrees(compass_values[2]) + 180
        
        self.distances['dm'] = self.dm_sensor.getValue() * 320
        self.distances['dr'] = self.dr_sensor.getValue() * 320
        self.distances['dl'] = self.dl_sensor.getValue() * 320
        self.distances['r45'] = self.r45_sensor.getValue() * 320
        self.distances['l45'] = self.l45_sensor.getValue() * 320
        
        self.wheel_positions['left'] = math.degrees(self.wheel_left_sensor.getValue())
        self.wheel_positions['right'] = math.degrees(self.wheel_right_sensor.getValue())
        
        self.lidar.robot_orientation = self.orientation
        self.lidar.get_hit_positions(self.position, self.orientation)
    
    def print_sensor_data(self):
        print('=== POSITION & ORIENTATION ===')
        print(f"Position - X: {self.position['x']:.2f} cm, Y: {self.position['y']:.2f} cm, Z: {self.position['z']:.2f} cm")
        print(f"Orientation - Roll: {self.orientation['roll']:.2f}°, Pitch: {self.orientation['pitch']:.2f}°, Yaw: {self.orientation['yaw']:.2f}°")
        
        print(f"\n=== TILE INFORMATION ===")
        print(f"Current tile: ({self.current_tile['x']}, {self.current_tile['z']})")
        
        print('\n=== DISTANCE SENSORS ===')
        print(f"DM (middle): {self.distances['dm']:.2f}")
        print(f"DR (right): {self.distances['dr']:.2f}")
        print(f"DL (left): {self.distances['dl']:.2f}")
        print(f"R45 (right diagonal): {self.distances['r45']:.2f}")
        print(f"L45 (left diagonal): {self.distances['l45']:.2f}")
        
        print('\n=== LiDAR DATA ===')
        lidar_params = self.lidar.get_lidar_params()
        print(f"Number of hit points: {len(self.lidar.hit_positions)}")
        
        if self.lidar.hit_positions:
            print("\nSample hit positions (first 5):")
            for i, pos in enumerate(self.lidar.hit_positions[:5]):
                print(f"  Hit {i+1}: X={pos[0]:.2f}, Z={pos[1]:.2f}")
        
        print('\n=== WHEEL POSITIONS ===')
        print(f"Left wheel: {self.wheel_positions['left']:.2f}°")
        print(f"Right wheel: {self.wheel_positions['right']:.2f}°")
        print('-' * 50)
    
    def set_orientation(self, target_angle):
        current_yaw = self.orientation['yaw']
        error = current_yaw - target_angle
        
        if error > 360:
            error -= 360
        elif error < 0:
            error += 360
        
        P = 0.2
        speed = P * error
        
        if abs(error) <= 0.1:
            speed = 0
        
        self.set_wheel_velocities(-speed, speed)
        return abs(error) <= 0.1
    
    def run(self):
        while self.robot.step(self.timestep) != -1:
            self.read_all_sensors()
            self.print_sensor_data()
            
            self.wheel_left.setVelocity(0)
            self.wheel_right.setVelocity(0)

            angle, distance = self.calculate_target_angle_distance(-60, -30)
            print(f"angle: {angle:.2f}, distance: {distance:.2f}")

controller = RobotController()
controller.run()
