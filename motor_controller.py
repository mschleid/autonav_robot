#!/usr/bin/env python3
"""
ROS 2 node that subscribes to geometry_msgs/Twist messages and drives the Robot hardware.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from jetbot import Robot


def clamp(value: float, min_value: float = -1.0, max_value: float = 1.0) -> float:
    return max(min_value, min(max_value, value))


class TwistDriveNode(Node):

    def __init__(self):
        super().__init__('twist_drive')
        self.robot = Robot()
        # Subscribe to the standard /cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.get_logger().info('TwistDriveNode initialized, waiting for /cmd_vel')

    def cmd_vel_callback(self, msg: Twist) -> None:
        linear = msg.linear.x
        angular = msg.angular.z

        left = clamp(linear - angular)
        right = clamp(linear + angular)

        self.robot.set_motors(left, right)

        self.get_logger().debug(
            f"cmd_vel received: linear={linear:.2f}, angular={angular:.2f} -> "
            f"left={left:.2f}, right={right:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TwistDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down, stopping robot motors')
        node.robot.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

