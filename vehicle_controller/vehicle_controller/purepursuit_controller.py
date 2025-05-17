#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, JointState 
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import math
import json
from dataclasses import dataclass, asdict
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
import time
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Point
@dataclass
class Point2D:
    x: float
    y: float

    def distance(self, other):
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)

class AutonomousRacingCar(Node):
    def __init__(self):
        super().__init__('autonomous_racing_car')
        
        # Centerline detector parameters
        self.ANGLE_MIN = -math.pi
        self.ANGLE_MAX = math.pi
        self.RANGE_MIN = 0.1
        self.RANGE_MAX = 15.0
        self.TRACK_WIDTH_MIN = 0.9
        self.TRACK_WIDTH_MAX = 3.0
        self.DUCT_DIAMETER = 0.33
        self.MAX_CENTERLINE_HISTORY = 100
        self.data=None
        self.path_x=0
        self.path_y=0
        self.last_index =0
        self.recent_centerline = []
        self.declare_parameter('time_interval',0.01)
        self.time_interval = float(self.get_parameter('time_interval').get_parameter_value().double_value)
        self.curvature=0.0
        
        # Pure pursuit parameters
        self.current_x_pose = 0
        self.current_y_pose = 0
        self.heading = 0
        self.lookahead_distance = 1.5
        self.vehicle_length = 0.5
        self.STEER_LIMIT = 1.0
        self.DRIVE_LIMIT = 0.05
        self.current_steer=0
        
        # Encoder parameters
        self.left_ticks = 0
        self.right_ticks = 0
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0
        self.distance_per_tick = (2 * math.pi * 0.0590) / (16 * 120)
        self.wheel_base = 0.3240
        self.steering_msg = Float32()
        #frames
        self.flw = 'front_left_wheel'
        self.frw = 'front_right_wheel'
        self.blw = 'rear_left_wheel'
        self.brw = 'rear_right_wheel'
        self.ff = 'world'

        # Transform Listener initialization
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)	
        
        qos = QoSProfile(depth=5)
        
        # Subscriptions
        self.create_subscription(Imu, "/autodrive/f1tenth_1/imu", self.imu_callback, qos_profile=qos)     

        # Publishers
        self.pub_steering_command = self.create_publisher(Float32, '/autodrive/f1tenth_1/steering_command', qos_profile=qos)
        self.pub_throttle_command = self.create_publisher(Float32, '/autodrive/f1tenth_1/throttle_command', qos_profile=qos)
        #self.pub_odometry = self.create_publisher(Odometry, '/odom', qos_profile=qos)
        
        # Timer for control loop
        time.sleep(1)  # Wait for frames to  initialize
        #self.timer = self.create_timer(self.time_interval, self.compute)

        self.get_logger().info('Autonomous Racing Car node initialized')

    def compute(self):
            try:
                t_flw = self.tf_buffer.lookup_transform(self.ff, self.flw, rclpy.time.Time())
                t_frw = self.tf_buffer.lookup_transform(self.ff, self.frw, rclpy.time.Time())
                t_blw = self.tf_buffer.lookup_transform(self.ff, self.blw, rclpy.time.Time())
                t_brw = self.tf_buffer.lookup_transform(self.ff, self.brw, rclpy.time.Time())
            except TransformException as e:
                # Check the buffer length in code
                self.get_logger().warn(f'Transform error: {e}')
                return
            # Calculate front axle midpoint
            xf0 = (t_flw.transform.translation.x + t_frw.transform.translation.x) / 2
            yf0 = (t_flw.transform.translation.y + t_frw.transform.translation.y) / 2
            z0 = (t_flw.transform.translation.z + t_frw.transform.translation.z) / 2
            # Calculate back axle midpoint
            xb0 = (t_blw.transform.translation.x + t_brw.transform.translation.x) / 2
            yb0 = (t_blw.transform.translation.y + t_brw.transform.translation.y) / 2
            
            self.current_x_pose = (xf0 + xb0)/2
            self.current_y_pose = (yf0 + yb0)/2
            self.get_logger().info(f'crrent x pos:{self.current_x_pose},current y pos:{self.current_y_pose}')
            self.CalcWayPointTarget()
            self.pure_pursuit_controller()
            #self.save_waypoints_to_json()
            
#************************************************************************************************************************************
    def save_waypoints_to_json(self):
        # Define the file path where the waypoints will be saved
        json_file_path = '/home/autodrive_devkit/src/vehicle_controller/vehicle_controller/x.json'

        # Create a new waypoint dictionary for the current position
        waypoint = {"x": self.current_x_pose, "y": self.current_y_pose}

        try:
            # Load existing waypoints from the file if it exists
            with open(json_file_path, 'r') as file:
                waypoints = json.load(file)
        except FileNotFoundError:
            # If the file does not exist, initialize an empty list
            waypoints = []

        # Append the new waypoint to the list
        waypoints.append(waypoint)

        # Save the updated waypoints list back to the file
        with open(json_file_path, 'w') as file:
            json.dump(waypoints, file, indent=4)

        # Log the save operation
        self.get_logger().info(f"Waypoint saved: {waypoint}")

#************************************************************************************************************************************
    def imu_callback(self, msg: Imu):
        quaternion = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        euler_angles = euler_from_quaternion(quaternion)
        self.heading = euler_angles[2]
        self.compute()
#************************************************************************************************************************************
    def left_encoder_callback(self, msg: JointState):
        if msg.position:
            self.left_ticks = msg.position[0]
#************************************************************************************************************************************
    def right_encoder_callback(self, msg: JointState):
        if msg.position:
            self.right_ticks = msg.position[0]
            
#************************************************************************************************************************************

    # def ips_callback(self,msg :Point):
    #     self.current_x_pose= msg.x
    #     self.current_y_pose= msg.y
    #     self.save_waypoints_to_json()
#************************************************************************************************************************************
    def pure_pursuit_controller(self):
        dx = self.path_x - self.current_x_pose
        dy = self.path_y - self.current_y_pose
        local_x = dx * math.cos(-self.heading) - dy * math.sin(-self.heading)
        local_y = dx * math.sin(-self.heading) + dy * math.cos(-self.heading)
        if local_x != 0:
            self.curvature = 2 * local_y / (self.lookahead_distance ** 2)
        else:
            self.curvature = 0
        self.steering_angle = math.atan(self.vehicle_length*self.curvature)
        self.current_steer = max(-self.STEER_LIMIT, min(self.STEER_LIMIT, self.steering_angle))
        #self.get_logger().info(f"curvature found : {self.curvature}")
        self.publishFunctionSteer()

        # Calculate and publish throttle command
        throttle = self.calculate_throttle(self.curvature)
        throttle_msg = Float32()
        throttle_msg.data = float(throttle + .05)
        self.pub_throttle_command.publish(throttle_msg)
#************************************************************************************************************************************
    def CalcWayPointTarget(self):
        with open('/home/autodrive_devkit/src/vehicle_controller/vehicle_controller/x.json', 'r') as file:
            self.data = json.load(file)

        closest_point = None
        min_distance = float('inf')
        

        search_window = 10
        start_index = self.last_index 
        end_index = min(start_index + search_window, len(self.data))


        if end_index >= len(self.data):
            start_index = max(0, len(self.data) - search_window)
            end_index = len(self.data)

        for i in range(start_index, end_index):
            point = self.data[i]
            distance = math.sqrt((point["x"] - self.current_x_pose)**2 + (point["y"] - self.current_y_pose)**2)
            
            if distance < min_distance and distance >= self.lookahead_distance:
                min_distance = distance
                closest_point = point
                self.last_index = i  

        if closest_point is None:
            for i in range(len(self.data)):
                point = self.data[i]
                distance = math.sqrt((point["x"] - self.current_x_pose)**2 + (point["y"] - self.current_y_pose)**2)
                if distance < min_distance:
                    min_distance = distance
                    closest_point = point
                    self.last_index = i
        if closest_point is not None:
            self.path_x = closest_point["x"]
            self.path_y = closest_point["y"]
            self.get_logger().info(f"Target point found at x: {self.path_x}, y: {self.path_y}")
        else:
            self.last_index = 0  

#************************************************************************************************************************************
    def calculate_throttle(self, curvature):
        max_throttle = 0.1
        min_throttle = 0.03
        max_curvature = 1.0

        throttle = max(min_throttle, max_throttle - (abs(curvature) / max_curvature) * (max_throttle - min_throttle))
        if(abs(self.curvature)>0 and abs(self.curvature)<.2):
            pass

        return throttle
#************************************************************************************************************************************
    def publishFunctionSteer(self):
        self.steering_msg.data = float(self.current_steer)
        self.pub_steering_command.publish(self.steering_msg)
#************************************************************************************************************************************** 
def main(args=None):
    rclpy.init(args=args)
    node = AutonomousRacingCar()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

