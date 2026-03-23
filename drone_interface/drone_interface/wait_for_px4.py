#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry
import sys

class WaitForPX4(Node):
    def __init__(self):
        super().__init__('wait_for_px4')
        self.timer = self.create_timer(1.0, self.check_topic)
        self.get_logger().info("Waiting for PX4 odometry...")

    def check_topic(self):
        topics = self.get_topic_names_and_types()
        topic_names = [t[0] for t in topics]

        if '/fmu/out/vehicle_odometry' in topic_names:
            self.get_logger().info('PX4 topic detected. Exiting...')
            sys.exit(0)
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = WaitForPX4()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
