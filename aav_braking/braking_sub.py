import rclpy
from rclpy.node import Node
from aav_interfaces.msg import BrakingFeedbackInterface

class BrakePosReader(Node):
    def __init__(self):
        super().__init__("Brake_Pos_Sub")
        #Subscribption needs a callback to when a message is received
        self.pose_subscriber = self.create_subscription(BrakingFeedbackInterface,"/aav/brake/brake_pos", self.brake_read_callback, 10)
        self.get_logger().info("Brake Pos Sub Node started")

    def brake_read_callback(self, msg: BrakingFeedbackInterface):
        self.get_logger().info(str(msg.braking_fb))


def main(args=None):
    rclpy.init(args=args)
    node = BrakePosReader()
    rclpy.spin(node)
    rclpy.shutdown()