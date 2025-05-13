# Goal detection node
# Stops the robot when a goal is detected in the vision topic

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class GoalNode(Node):
    def __init__(self):
        super().__init__('goal_node')
        self.subscription = self.create_subscription(
            String,
            '/object_counts',
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def listener_callback(self, msg):
        if 'goal' in msg.data:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = GoalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
