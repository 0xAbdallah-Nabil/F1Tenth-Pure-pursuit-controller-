import keyboard
import rclpy
from std_msgs.msg import Float32 # Float32 message class



class my_node:
    def __init__(self):
        print("Abdallah Nabil")
        rclpy.init()
        node = rclpy.create_node('teleop_controller')
        self.pub_steering_command = node.create_publisher(Float32, '/autodrive/f1tenth_1/steering_command', 10)
        self.pub_throttle_command = node.create_publisher(Float32, '/autodrive/f1tenth_1/throttle_command', 10)
        ##self.sub_speed =node.create_subscription(Float32, '/autodrive/f1tenth_1/speed',self.listener_callback(),10)
        ##self.sub_speed
        ##init
        self.DRIVE_LIMIT = 0.5
        self.STEER_LIMIT = 1.0
        self.DRIVE_STEP_SIZE = 0.05
        self.STEER_STEP_SIZE = 0.05
        self.throttle_msg = Float32()
        self.steering_msg = Float32()
        self.current_steer=0.0
        self.current_throttle=0.0
        ##publish
    def publishFunctionSpeed(self):
        self.throttle_msg.data = float(self.current_throttle)
        self.pub_throttle_command.publish(self.throttle_msg)
    def publishFunctionSteer(self):
        self.steering_msg.data = float(self.current_steer)
        self.pub_steering_command.publish(self.steering_msg)   
        ##subscriber
    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data) 
        ##control functions for throttle
    def increase_Throttle(self):
        print("increase speed")
        if(self.current_throttle<(self.DRIVE_LIMIT)):
            self.current_throttle=self.current_throttle+self.DRIVE_STEP_SIZE
        elif(self.current_throttle>=(self.DRIVE_LIMIT)):
            self.current_throttle=self.DRIVE_LIMIT
        self.publishFunctionSpeed()
    def decrease_Throttle(self):
        print("decrease speed")
        if(self.current_throttle>(-self.DRIVE_LIMIT)):
            self.current_throttle=self.current_throttle-self.DRIVE_STEP_SIZE
        elif(self.current_throttle<=(-self.DRIVE_LIMIT)):
            self.current_throttle=(-self.DRIVE_LIMIT)
        self.publishFunctionSpeed()
        ##control functions for steering
    def increase_Steering(self):
        if(self.current_steer<(self.STEER_LIMIT)):
            self.current_steer=self.current_steer+self.STEER_STEP_SIZE
        elif(self.current_steer>=(self.STEER_LIMIT)):
            self.current_steer=self.STEER_LIMIT
        self.publishFunctionSteer()
    def decrease_Steering(self):
        if(self.current_steer>(-self.STEER_LIMIT)):
            self.current_steer=self.current_steer-self.STEER_STEP_SIZE
        elif(self.current_steer<=(-self.STEER_LIMIT)):
            self.current_steer=(-self.STEER_LIMIT)
        self.publishFunctionSteer()
    def breaks(self):
        self.current_throttle=0.0
        self.current_steer=0.0
        self.publishFunctionSpeed()
        self.publishFunctionSteer()
        ##keyborad key detect
    def keyboardInput(self):
        keyboard.add_hotkey("w",self.increase_Throttle)
        keyboard.add_hotkey("s",self.decrease_Throttle)
        keyboard.add_hotkey('a',self.increase_Steering)
        keyboard.add_hotkey("d",self.decrease_Steering)
        keyboard.add_hotkey("z",self.breaks)
        keyboard.wait('esc') 
def main():
    x = my_node()
    x.keyboardInput()
    ##rclpy.spin(x)
    ##x.destroy_node()
    ##rclpy.shutdown()
if __name__ == '__main__':
    main()
        