import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        # Subscribes to processed object data
        self.subscription = self.create_subscription(String, '/object_counts', self.listener_callback, 10)
        # Publishes velocity commands
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def listener_callback(self, msg):
        twist = Twist()
        # Adjust velocity based on sign detection
        if 'stop' in msg.data:
            twist.linear.x = 0.0
        elif 'slow' in msg.data:
            twist.linear.x = 0.1
        elif 'speed' in msg.data:
            twist.linear.x = 0.3

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
