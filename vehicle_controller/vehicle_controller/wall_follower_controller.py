import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
import math
from geometry_msgs.msg import Point
class WallFollowNode(Node):
    def __init__(self):
        super().__init__('wall_follower_controller')

        self.speed_publisher_ = self.create_publisher(Float32, '/autodrive/f1tenth_1/throttle_command', 10)
        self.steering_publisher_ = self.create_publisher(Float32, '/autodrive/f1tenth_1/steering_command', 10)
        self.scan_subscription_ = self.create_subscription(LaserScan, '/autodrive/f1tenth_1/lidar', self.scan_callback, 10)
        self.Points_subscription=self.create_subscription(Point,"/autodrive/f1tenth_1/ips",self.callback_points,10)

        self.desired_wall_distance_ = 1.0
        self.max_steering_angle_ = 1

        self.max_speed_ = 0.18
        self.min_speed_ = 0.1
        self.safe_distance_ = 4.0
        self.stop_distance_ = 2.0
        self.steering_sensitivity_ = 3.8
        self.point_dict={"x": 0.0, "y": 0.0, "z": 0.0}
        self.kp_ = 1.0
        self.ki_ = 0.2
        self.kd_ = 0.06
        self.integral_ = 0.0
        self.previous_error_ = 0.0

        self.last_callback_time_ = self.get_clock().now()

        self.get_logger().info("WallFollowNode has been started.")
    
    def callback_points(self,point:Point):
        self.get_logger().info(f"Point: x={point.x}, y={point.y}, z={point.z}")
        self.point_dict = {"x": point.x, "y": point.y, "z": point.z}
    def scan_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_callback_time_).nanoseconds / 1e9     
        self.last_callback_time_ = current_time
        

        front_range = min(msg.ranges[520:600])
        left_range = min(msg.ranges[420:480])
        right_range = min(msg.ranges[600:660])

        if left_range < right_range:
            error = self.desired_wall_distance_ - left_range

        else:
            error = right_range - self.desired_wall_distance_

        self.integral_ += error * dt
        self.integral_ = max(min(self.integral_, 1.0), -1.0)
        derivative = (error - self.previous_error_) / dt
        steering_angle = self.kp_ * error + self.ki_ * self.integral_ + self.kd_ * derivative
        self.previous_error_ = error

        steering_angle = max(min(steering_angle, self.max_steering_angle_), -self.max_steering_angle_)

        speed = self.calculate_speed(front_range, abs(steering_angle))

        self.publish_commands(speed, steering_angle)

    def calculate_speed(self, front_distance, steering_angle):
        distance_factor = (front_distance - self.stop_distance_) / (self.safe_distance_ - self.stop_distance_)
        distance_factor = max(min(distance_factor, 1.0), 0.0)
        distance_speed = self.min_speed_ + (self.max_speed_ - self.min_speed_) * distance_factor

        angle_factor = 1.0 - (abs(steering_angle) / self.max_steering_angle_) ** self.steering_sensitivity_
        angle_factor = max(min(angle_factor, 1.0), 0.0)
        angle_speed = self.min_speed_ + (self.max_speed_ - self.min_speed_) * angle_factor

        speed = min(distance_speed, angle_speed)

        return max(min(speed, self.max_speed_), self.min_speed_)

    def publish_commands(self, speed, steering_angle):
        speed_msg = Float32()
        speed_msg.data = speed
        self.speed_publisher_.publish(speed_msg)

        steering_msg = Float32()
        steering_msg.data = float(steering_angle)
        self.steering_publisher_.publish(steering_msg)

        self.get_logger().info(f'Published commands: speed={speed:.2f}, steering_angle={math.degrees(steering_angle):.2f}°')

def main(args=None):
    rclpy.init(args=args)
    node = WallFollowNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
