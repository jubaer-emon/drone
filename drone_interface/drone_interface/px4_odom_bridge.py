#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Time
from tf2_ros import TransformBroadcaster
import math

class Px4OdomBridge(Node):
    def __init__(self):
        super().__init__('px4_odom_bridge')
        self.sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.callback,
            qos_profile_sensor_data
        )
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos_profile_sensor_data)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Read once — safe since use_sim_time is set before node spin
        self.use_sim_time = self.get_parameter('use_sim_time').value
        self.get_logger().info(
            f"Stamping with: {'sim /clock' if self.use_sim_time else 'PX4 hardware time'}"
        )

    @staticmethod
    def _ned_to_enu(q_ned_wxyz):
        """
        Convert quaternion from NED body frame to ENU body frame.
        
        The NED->ENU frame transform is a fixed rotation:
        - 90° rotation around Z (maps N->E, E->N)  
        - 180° rotation around X (maps D->U, flips Z)
        Combined: rotate by q_frame = [w=0, x=√2/2, y=√2/2, z=0]
        
        q_enu = q_frame * q_ned
        """
        import math
        
        # q_frame for NED->ENU: 180° around X+Y axis
        # = [w=0, x=1/√2, y=1/√2, z=0]
        s = math.sqrt(0.5)
        fw, fx, fy, fz = 0.0, s, s, 0.0
        
        w1, x1, y1, z1 = fw, fx, fy, fz
        w2, x2, y2, z2 = q_ned_wxyz
        
        # Hamilton product q_frame * q_ned
        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2
        
        norm = math.sqrt(w*w + x*x + y*y + z*z)
        return [w/norm, x/norm, y/norm, z/norm]
    
    @staticmethod
    def _ned_to_enu_quat_old(q_ned):
        """NED [w,x,y,z] → ENU [w,x,y,z]. Closed-form: swap x/y, negate z."""
        w, x, y, z = q_ned
        q = [w, y, x, -z]
        norm = math.sqrt(sum(v * v for v in q))
        return [v / norm for v in q] if norm > 1e-9 else [1.0, 0.0, 0.0, 0.0]

    @staticmethod
    def _frd_ned_to_flu_enu(q):
        """
        Convert PX4 quaternion (FRD->NED) to ROS quaternion (FLU->ENU).
        
        q is [w, x, y, z] representing FRD body -> NED world
        
        Two fixed rotations needed:
        1. NED world -> ENU world:  q1 = [0, 1/√2, 1/√2, 0]  (180° around X+Y)
        2. FRD body -> FLU body:    q2 = [0, 1, 0, 0]          (180° around X)
        
        result = q1 * q * q2_conjugate
        """
        import math
        
        def qmul(a, b):
            w1,x1,y1,z1 = a
            w2,x2,y2,z2 = b
            return [
                w1*w2 - x1*x2 - y1*y2 - z1*z2,
                w1*x2 + x1*w2 + y1*z2 - z1*y2,
                w1*y2 - x1*z2 + y1*w2 + z1*x2,
                w1*z2 + x1*y2 - y1*x2 + z1*w2,
            ]
        
        def qnorm(q):
            n = math.sqrt(sum(v*v for v in q))
            return [v/n for v in q] if n > 1e-9 else [1,0,0,0]
        
        s = math.sqrt(0.5)
        
        # NED->ENU world frame rotation
        q_ned_to_enu = [0.0, s, s, 0.0]
        
        # FRD->FLU body frame rotation (180° around X)
        # conjugate of [0,1,0,0] is [0,-1,0,0]
        q_frd_to_flu_conj = [0.0, -1.0, 0.0, 0.0]
        
        result = qmul(qmul(q_ned_to_enu, q), q_frd_to_flu_conj)
        return qnorm(result)

    def callback(self, msg: VehicleOdometry):
        # ── Timestamp ──────────────────────────────────────────────
        # if self.use_sim_time:
        stamp = self.get_clock().now().to_msg()
        # else:
        # stamp = Time(
        #     sec=int(msg.timestamp_sample // 1_000_000),
        #     nanosec=int((msg.timestamp_sample % 1_000_000) * 1000)
        # )

        # ── Pose ───────────────────────────────────────────────────
        # NED → ENU position: (N,E,D) → (E,N,U)
        px, py, pz = msg.position[1], msg.position[0], -msg.position[2]

        # Quaternion: pass as [w,x,y,z] — verify field order first!
        qw, qx, qy, qz = self._frd_ned_to_flu_enu(
            [msg.q[0], msg.q[1], msg.q[2], msg.q[3]]
        )

        # ── Twist (body frame) ─────────────────────────────────────
        # NED body → ENU body: swap x/y, negate z
        vx, vy, vz = msg.velocity[1],         msg.velocity[0],         -msg.velocity[2]
        wx, wy, wz = msg.angular_velocity[1], msg.angular_velocity[0], -msg.angular_velocity[2]

        # ── Odometry message ───────────────────────────────────────
        odom = Odometry()
        odom.header.stamp    = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'

        odom.pose.pose.position.x    = float(px)
        odom.pose.pose.position.y    = float(py)
        odom.pose.pose.position.z    = float(pz)
        odom.pose.pose.orientation.w = float(qw)
        odom.pose.pose.orientation.x = float(qx)
        odom.pose.pose.orientation.y = float(qy)
        odom.pose.pose.orientation.z = float(qz)

        odom.twist.twist.linear.x  = float(vx)
        odom.twist.twist.linear.y  = float(vy)
        odom.twist.twist.linear.z  = float(vz)
        odom.twist.twist.angular.x = float(wx)
        odom.twist.twist.angular.y = float(wy)
        odom.twist.twist.angular.z = float(wz)

        # Diagonal covariance (row-major 6×6: x,y,z,roll,pitch,yaw)
        odom.pose.covariance[0]  = 0.01   # x
        odom.pose.covariance[7]  = 0.01   # y
        odom.pose.covariance[14] = 0.01   # z
        odom.pose.covariance[21] = 0.1    # roll
        odom.pose.covariance[28] = 0.1    # pitch
        odom.pose.covariance[35] = 0.1    # yaw

        odom.twist.covariance[0]  = 0.1   # vx
        odom.twist.covariance[7]  = 0.1   # vy
        odom.twist.covariance[14] = 0.1   # vz
        odom.twist.covariance[21] = 0.2   # wx
        odom.twist.covariance[28] = 0.2   # wy
        odom.twist.covariance[35] = 0.2   # wz

        self.odom_pub.publish(odom)

        # ── TF broadcast ───────────────────────────────────────────
        t = TransformStamped()
        t.header.stamp    = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_link'
        t.transform.translation.x = float(px)
        t.transform.translation.y = float(py)
        t.transform.translation.z = float(pz)
        t.transform.rotation      = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = Px4OdomBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()