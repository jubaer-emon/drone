#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data

from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Time
from tf2_ros import TransformBroadcaster

import numpy as np
from scipy.spatial.transform import Rotation as R


class Px4OdomBridge(Node):
    def __init__(self):
        super().__init__('px4_odom_bridge')

        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.callback,
            qos_profile_sensor_data
        )

        self.odom_pub = self.create_publisher(
            Odometry, '/odom', odom_qos
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.use_sim_time = self.get_parameter('use_sim_time').value
        self.get_logger().info(
            f"Stamping with: {'sim /clock' if self.use_sim_time else 'PX4 time'}"
        )

    # ────────────────────────────────────────────────
    # Frame conversions
    # ────────────────────────────────────────────────

    @staticmethod
    def ned_to_enu_position(p):
        """NED → ENU position"""
        return [p[1], p[0], -p[2]]

    # @staticmethod
    # def aircraft_to_baselink_orientation(quat):
    #     """Rotate quaternion from PX4 aircraft frame (FRD) to ROS base_link frame (FLU)"""
    #     r = R.from_quat([quat[1], quat[2], quat[3], quat[0]])  # x,y,z,w
    #     r_rot = R.from_euler('x', np.pi)  # 180° around X
    #     r_out = r_rot * r
    #     q_out = r_out.as_quat()
    #     return np.array([q_out[3], q_out[0], q_out[1], q_out[2]])  # w,x,y,z

    # @staticmethod
    # def ned_to_enu_orientation(quat):
    #     """Rotate quaternion from NED → ENU frame"""
    #     r = R.from_quat([quat[1], quat[2], quat[3], quat[0]])
    #     # NED→ENU: +90° Z, +180° X
    #     r_rot = R.from_euler('z', np.pi/2) * R.from_euler('x', np.pi)
    #     r_out = r_rot * r
    #     q_out = r_out.as_quat()
    #     return np.array([q_out[3], q_out[0], q_out[1], q_out[2]])

    # @staticmethod
    # def px4_to_ros_orientation(quat_px4):
    #     """Full conversion: Aircraft frame → Base_link → NED→ENU"""
    #     q_baselink = Px4OdomBridge.aircraft_to_baselink_orientation(quat_px4)
    #     q_ros = Px4OdomBridge.ned_to_enu_orientation(q_baselink)
    #     return q_ros

    @staticmethod
    def px4_to_ros_orientation(quat_px4):
        r = R.from_quat([quat_px4[1], quat_px4[2], quat_px4[3], quat_px4[0]])

        # Only Z rotation needed after simplification
        r_rot = R.from_euler('z', np.pi/2)

        r_out = r_rot * r
        q_out = r_out.as_quat()

        return np.array([q_out[3], q_out[0], q_out[1], q_out[2]])
    
    @staticmethod
    def frd_to_flu_angular_velocity(w):
        """Convert angular velocity FRD → FLU"""
        rot = R.from_euler('x', np.pi)
        return rot.apply(w)

    # ────────────────────────────────────────────────
    # Callback
    # ────────────────────────────────────────────────

    def callback(self, msg: VehicleOdometry):

        # ── Time ─────────────────────────────────────
        if self.use_sim_time:
            stamp = self.get_clock().now().to_msg()
        else:
            stamp = Time(
                sec=int(msg.timestamp_sample // 1_000_000),
                nanosec=int((msg.timestamp_sample % 1_000_000) * 1000)
            )

        # ── Position ─────────────────────────────────
        px, py, pz = self.ned_to_enu_position(msg.position)

        # ── Orientation ──────────────────────────────
        qw, qx, qy, qz = self.px4_to_ros_orientation(msg.q)

        # ── Linear velocity ──────────────────────────
        vx, vy, vz = self.ned_to_enu_position(msg.velocity)

        # ── Angular velocity ─────────────────────────
        ang = np.array(msg.angular_velocity)
        wx, wy, wz = self.frd_to_flu_angular_velocity(ang)

        # ── Odometry message ─────────────────────────
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = float(px)
        odom.pose.pose.position.y = float(py)
        odom.pose.pose.position.z = float(pz)

        odom.pose.pose.orientation.w = float(qw)
        odom.pose.pose.orientation.x = float(qx)
        odom.pose.pose.orientation.y = float(qy)
        odom.pose.pose.orientation.z = float(qz)

        odom.twist.twist.linear.x = float(vx)
        odom.twist.twist.linear.y = float(vy)
        odom.twist.twist.linear.z = float(vz)

        odom.twist.twist.angular.x = float(wx)
        odom.twist.twist.angular.y = float(wy)
        odom.twist.twist.angular.z = float(wz)

        # Covariance
        odom.pose.covariance[0]  = 0.01
        odom.pose.covariance[7]  = 0.01
        odom.pose.covariance[14] = 0.01
        odom.pose.covariance[21] = 0.1
        odom.pose.covariance[28] = 0.1
        odom.pose.covariance[35] = 0.1

        odom.twist.covariance[0]  = 0.1
        odom.twist.covariance[7]  = 0.1
        odom.twist.covariance[14] = 0.1
        odom.twist.covariance[21] = 0.2
        odom.twist.covariance[28] = 0.2
        odom.twist.covariance[35] = 0.2

        self.odom_pub.publish(odom)

        # ── TF ──────────────────────────────────────
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = float(px)
        t.transform.translation.y = float(py)
        t.transform.translation.z = float(pz)
        t.transform.rotation = odom.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = Px4OdomBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()