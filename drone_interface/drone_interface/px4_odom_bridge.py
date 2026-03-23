#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Time

from tf2_ros import TransformBroadcaster
import math

def ned_to_enu_quaternion(q):
    # PX4 gives (w, x, y, z)
    w, x, y, z = q

    # Rotation: 180° about X + 90° about Z
    # Precomputed rotation quaternion
    # q_rot = [0, 0.7071, 0.7071, 0]

    rw = 0.0
    rx = 0.70710678
    ry = 0.70710678
    rz = 0.0

    # Quaternion multiply: q_rot * q
    qw = rw*w - rx*x - ry*y - rz*z
    qx = rw*x + rx*w + ry*z - rz*y
    qy = rw*y - rx*z + ry*w + rz*x
    qz = rw*z + rx*y - ry*x + rz*w

    return [qw, qx, qy, qz]

class Px4OdomBridge(Node):

    def __init__(self):
        super().__init__('px4_odom_bridge')

        self.sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.callback,
            qos_profile_sensor_data
        )

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        if self.get_parameter('use_sim_time').value:
            self.get_logger().info("Will use /clock time")
        else:
            self.get_logger().info("Will use PX4 time")

    def callback(self, msg):
        if self.get_parameter('use_sim_time').value:
            stamp = self.get_clock().now().to_msg()
        else:
            stamp = Time()
            stamp.sec = int(msg.timestamp_sample // 1_000_000)
            stamp.nanosec = int((msg.timestamp_sample % 1_000_000) * 1000)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Position
        # Corrected ENU position
        odom.pose.pose.position.x = float( msg.position[0])
        odom.pose.pose.position.y = float( msg.position[1])  # no negative
        odom.pose.pose.position.z = float(-msg.position[2])  # z down→up

        # Orientation

        q_enu = ned_to_enu_quaternion(msg.q)
        
        odom.pose.pose.orientation.x = float(q_enu[1])
        odom.pose.pose.orientation.y = float(q_enu[2])
        odom.pose.pose.orientation.z = float(q_enu[3])
        odom.pose.pose.orientation.w = float(q_enu[0])

        # Velocity
        odom.twist.twist.linear.x = float( msg.velocity[0])
        odom.twist.twist.linear.y = float( msg.velocity[1])
        odom.twist.twist.linear.z = float(-msg.velocity[2])

        self.odom_pub.publish(odom)

        # TF
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = odom.pose.pose.position.x
        t.transform.translation.y = odom.pose.pose.position.y
        t.transform.translation.z = odom.pose.pose.position.z

        t.transform.rotation = odom.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = Px4OdomBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
