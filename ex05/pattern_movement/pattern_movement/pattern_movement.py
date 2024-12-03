import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import numpy as np


class MovementsNode(Node):
    def __init__(self):
        super().__init__('movements_node')
        self.publisher_ = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.twist_msg = Twist()
        timer_period = 0.1  # seconds
        self.linear = 1.0
        self.angular = 0.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.make_line()
        self.rotate()

    def rotate(self):
        self.twist_msg.angular.z = 0.5
        self.publisher_.publish(self.twist_msg)
        time.sleep(np.pi)

    def make_line(self):
        self.twist_msg.linear.x = 1.0
        self.twist_msg.angular.z = 0.0
        self.publisher_.publish(self.twist_msg)
        time.sleep(2.0)


def main(args=None):
    rclpy.init(args=args)
    node = MovementsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()