import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import math

class NavNode(Node):
    def __init__(self):
        super().__init__('nav_node')

        # Use hardcoded goal initially
        self.goal_x = 2.0
        self.goal_y = 2.0
        self.use_rviz_goal = False  # Flag to switch between RViz and hardcoded mode

        # Subscriber for RViz 2D Nav Goal (PoseStamped message)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        # Subscribe to odometry updates
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def goal_callback(self, msg):
        # Handle incoming goal from RViz (PoseStamped)
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.use_rviz_goal = True  # Activate RViz navigation
        self.get_logger().info(f"Received goal from RViz: ({self.goal_x:.2f}, {self.goal_y:.2f})")

    def odom_callback(self, msg):
        # Use odometry to get the robot's current position
        pos = msg.pose.pose.position
        dx = self.goal_x - pos.x
        dy = self.goal_y - pos.y
        distance = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)

        twist = Twist()

        # Navigate only if the goal (hardcoded or RViz) is far enough away
        if distance > 0.1:
            twist.linear.x = 0.15
            twist.angular.z = angle_to_goal
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = NavNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
