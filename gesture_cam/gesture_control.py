#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import cv2 as cv

class GestureControl(Node): 
    def __init__(self):
        super().__init__("gesture_control") 
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gesture_subscriber_ = self.create_subscription(String, '/gesture_recog', self.callback_motion, 10)
        self.subscriptions
        
    def callback_motion(self, msg):
        self.cmd = Twist()
        gesture = msg.data
        if gesture == "go":
            self.cmd.linear.x = 1.0
            self.cmd.angular.z = 0.0
        elif gesture == "reverse":
            self.cmd.linear.x = -1.0
            self.cmd.angular.z = 0.0
        elif gesture == "right":
            self.cmd.linear.x = 0.1
            self.cmd.angular.z = 0.5
        elif gesture == "left":
            self.cmd.linear.x = 0.1
            self.cmd.angular.z = -0.5
        elif gesture == "stop":
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
        else:
            self.cmd.linear.x = 0.0
            
        self.get_logger().info(f'Gesture received: {gesture}')
        self.publisher_.publish(self.cmd)

def main(args=None):
    rclpy.init(args=args)
    node = GestureControl()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
