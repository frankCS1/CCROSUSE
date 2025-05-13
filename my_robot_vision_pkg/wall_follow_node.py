import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class WallFollowNode(Node):
    def __init__(self):
        super().__init__('wall_follow_node')
        # Subscribe to LIDAR scan data from the '/scan' topic
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        # Create a publisher to control robot movement via Twist messages
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def scan_callback(self, msg):
        # This method is triggered whenever new LaserScan data is received
        twist = Twist()
        front = msg.ranges[0]          # Distance measurement directly in front of the robot
        left = msg.ranges[90]          # Distance measurement to the left

        # Decision logic based on proximity to obstacles
        if front < 0.5:
            twist.angular.z = -0.5     # If an object is too close ahead, turn right
        elif left > 1.0:
            twist.angular.z = 0.5      # If the left is too far, veer left to follow wall
        else:
            twist.linear.x = 0.2       # Otherwise, move forward

        self.publisher.publish(twist)  # Send velocity command

def main(args=None):
    # Main function to initialize and run the ROS2 node
    rclpy.init(args=args)
    node = WallFollowNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
