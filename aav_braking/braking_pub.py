#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from aav_interfaces.msg import BrakingFeedbackInterface
import random
from std_msgs.msg import Int32
import threading

class BrakingPosFB(Node):
    def __init__(self):
        super().__init__("brake_feedback_node")
        #Publisher publishing data to topic
        self.brake_pos_pub = self.create_publisher(Int32,"/aav/brake/brake_pos", 5)
        self.timer = self.create_timer(0.5, self.send_brake_pos)
        self.get_logger().info("Brake Pos Publisher Node Started")
        self.user_input = 0

    def send_brake_pos(self):
        msg = Int32()
        msg.data = self.user_input
        self.brake_pos_pub.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')

    def take_user_input(self):
        while rclpy.ok():
            try:
                self.user_input = int(input("Enter: "))
            except ValueError:
                print("Invalid input")

def main(args=None):
    rclpy.init(args=args)
    node = BrakingPosFB()

    input_thread = threading.Thread(target=node.take_user_input)
    input_thread.start()

    rclpy.spin(node)
    rclpy.shutdown()
    input_thread.join()

if __name__ == '__main__':
    main()