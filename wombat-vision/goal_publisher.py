#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import time

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        self.publisher = self.create_publisher(Point, 'target_position', 10)
        self.get_logger().info('Goal Publisher has started.')

    def publish_goal(self, x, y):
        goal = Point()
        goal.x = x
        goal.y = y
        goal.z = 0.0
        self.publisher.publish(goal)
        self.get_logger().info(f'Published goal position: ({x}, {y})')

def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisher()  # Instantiate the GoalPublisher class
    try:
        node.publish_goal(2.0, 3.0)
    except KeyboardInterrupt:
        node.get_logger().info("Goal Publisher has been interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
