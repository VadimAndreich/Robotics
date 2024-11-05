import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

import math
import sys
import time


class MoveToGoalNode(Node):
    def __init__(self, goal_x, goal_y, goal_theta):
        super().__init__('move_to_goal_node')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Pose,
            'turtle1/pose',
            self.pose_callback,
            10)
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.goal_theta = goal_theta


    def pose_callback(self, msg):
        self.current_pose = msg
        self.move_to_goal()


    def move_to_goal(self):
        # Calculate the distance to the goal
        print(f"Current pose:\nx: {self.current_pose.x}, y: {self.current_pose.y}, theta: {self.current_pose.theta}")
        distance = math.sqrt((self.goal_x - self.current_pose.x) ** 2 + (self.goal_y - self.current_pose.y) ** 2)
        print(f"Distance to go: {distance}")

        # Calculate the angle to the goal
        angle_to_goal = math.atan2(self.goal_y - self.current_pose.y, self.goal_x - self.current_pose.x)

        # Calculate the difference between the current angle and the goal angle
        angle_diff = angle_to_goal - self.current_pose.theta

        # Normalize the angle difference to [-pi, pi]
        angle_diff = math.fabs(angle_diff) % (2 * math.pi) if angle_diff > 0 else -(math.fabs(angle_diff) % (2 * math.pi))
        print(angle_diff)

        twist = Twist()

        # Rotate to the goal position
        if math.fabs(distance) > 0.1 and math.fabs(angle_diff) > 0.1:
            twist.linear.x = 0.0
            twist.angular.z = angle_diff
            self.publisher_.publish(twist)
            time.sleep(1.2)

        # Move to the goal position
        if math.fabs(distance) > 0.1:
            twist.linear.x = distance
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            time.sleep(1.2)

        # Final rotate to the goal theta
        if self.current_pose.theta + angle_diff != self.goal_theta:
            twist.linear.x = 0.0
            angle_diff = self.goal_theta - (self.current_pose.theta + angle_diff)
            twist.angular.z = angle_diff
            self.publisher_.publish(twist)
            time.sleep(1.2)

        return


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 4:
        print("Usage: ros2 run move_to_goal move_to_goal_node <x> <y> <theta>")
        return

    goal_x = float(sys.argv[1])
    goal_y = float(sys.argv[2])
    goal_theta = float(sys.argv[3])

    node = MoveToGoalNode(goal_x, goal_y, goal_theta)

    rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()