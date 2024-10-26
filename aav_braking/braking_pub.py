#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from aav_interfaces.msg import BrakingFeedback
import random

class BrakingPosFB(Node):
    def __init__(self):
        super().__init__("brake_feedback_node")
        #Publisher publishing data to topic
        self.brake_pos_pub = self.create_publisher(BrakingFeedback,"/aav/brake/brake_pos", 5)
        self.timer = self.create_timer(0.5, self.send_brake_pos)
        self.get_logger().info("Brake Pos Publisher Node Started")

    def send_brake_pos(self):
        msg = BrakingFeedback()
        msg.braking_fb = random.randint(0,255)
        self.brake_pos_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = BrakingPosFB()
    rclpy.spin(node)
    rclpy.shutdown()