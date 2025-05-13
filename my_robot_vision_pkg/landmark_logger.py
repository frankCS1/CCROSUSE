import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import csv
from datetime import datetime

class LandmarkLogger(Node):
    def __init__(self):
        super().__init__('landmark_logger')
        # Subscribe to detection messages
        self.subscription = self.create_subscription(String, '/object_counts', self.listener_callback, 10)
        # Define path to log file
        self.log_file_path = '/home/ntu-user/landmark_log.csv'

    def listener_callback(self, msg):
        # When a message is received, log the time and contents to a CSV
        current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        with open(self.log_file_path, 'a') as f:
            writer = csv.writer(f)
            writer.writerow([current_time, msg.data])


def main(args=None):
    rclpy.init(args=args)
    node = LandmarkLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
