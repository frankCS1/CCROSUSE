import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TrafficNode(Node):
    def __init__(self):
        super().__init__('traffic_node')
        # Subscribe to object detection results
        self.subscription = self.create_subscription(String, '/object_counts', self.listener_callback, 10)

    def listener_callback(self, msg):
        # Log detection of specific traffic signs
        if 'stop' in msg.data:
            self.get_logger().info('Stop sign detected')
        elif 'speed' in msg.data:
            self.get_logger().info('Speed sign detected')
        elif 'slow' in msg.data:
            self.get_logger().info('Slow sign detected')


def main(args=None):
    rclpy.init(args=args)
    node = TrafficNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
