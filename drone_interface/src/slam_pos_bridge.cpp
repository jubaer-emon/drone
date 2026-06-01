#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <px4_ros_com/frame_transforms.h>

using std::placeholders::_1;

class TfToPx4Bridge : public rclcpp::Node
{
public:
    TfToPx4Bridge() : Node("tf_to_px4_bridge")
    {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Baro altitude passthrough — only needed if EKF2_HGT_REF=1 (baro)
        px4_odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry",
            rclcpp::QoS(5).reliability(rclcpp::ReliabilityPolicy::BestEffort),
            std::bind(&TfToPx4Bridge::px4OdomCallback, this, _1));

        px4_odom_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
            "/fmu/in/vehicle_visual_odometry",
            rclcpp::QoS(10)
                .reliability(rclcpp::ReliabilityPolicy::BestEffort)
                .durability(rclcpp::DurabilityPolicy::Volatile));

        // 50Hz — EKF2 needs consistent high-rate input to stay stable
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&TfToPx4Bridge::timerCallback, this));

        RCLCPP_INFO(this->get_logger(),
            "TF→PX4 bridge ready (no-mag mode, SLAM is heading reference)");
    }

private:
    void px4OdomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        current_px4_ned_z_ = msg->position[2];
        has_altitude_ = true;
    }

    void timerCallback()
    {
        if (!has_altitude_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Waiting for PX4 altitude...");
            return;
        }

        geometry_msgs::msg::TransformStamped tf;
        try {
            tf = tf_buffer_->lookupTransform(
                "map", "base_link",
                rclcpp::Time(0),
                rclcpp::Duration::from_seconds(0.05));
        }
        catch (const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "TF lookup failed: %s", ex.what());
            return;
        }

        // ── Position: SLAM ENU → PX4 NED ──────────────────────────────────
        Eigen::Vector3d enu_pos(
            tf.transform.translation.x,
            tf.transform.translation.y,
            0.0  // 2D LiDAR: no Z from SLAM
        );
        Eigen::Vector3d ned_pos =
            px4_ros_com::frame_transforms::enu_to_ned_local_frame(enu_pos);

        // ── Orientation: ROS baselink/ENU → PX4 aircraft/NED ──────────────
        // With MAG_TYPE=5, THIS is how PX4 learns its heading — send it correctly
        Eigen::Quaterniond enu_q(
            tf.transform.rotation.w,
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z
        );
        Eigen::Quaterniond ned_q =
            px4_ros_com::frame_transforms::ros_to_px4_orientation(enu_q);
        ned_q.normalize();

        // ── Build message ──────────────────────────────────────────────────
        auto msg = px4_msgs::msg::VehicleOdometry();

        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000ULL;
        msg.timestamp_sample =
            static_cast<uint64_t>(tf.header.stamp.sec)   * 1000000ULL +
            static_cast<uint64_t>(tf.header.stamp.nanosec) / 1000ULL;

        msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;

        msg.position[0] = static_cast<float>(ned_pos.x());
        msg.position[1] = static_cast<float>(ned_pos.y());
        msg.position[2] = current_px4_ned_z_;  // baro passthrough

        msg.q[0] = static_cast<float>(ned_q.w());
        msg.q[1] = static_cast<float>(ned_q.x());
        msg.q[2] = static_cast<float>(ned_q.y());
        msg.q[3] = static_cast<float>(ned_q.z());

        // Low variance = high trust in SLAM X/Y/yaw
        msg.position_variance[0] = 0.01f;
        msg.position_variance[1] = 0.01f;
        msg.position_variance[2] = 100.0f;  // Z untrusted (baro handles it)

        // Roll/pitch from SLAM is meaningless (2D) — high variance so EKF2 ignores
        msg.orientation_variance[0] = 1.0f;
        msg.orientation_variance[1] = 1.0f;
        msg.orientation_variance[2] = 0.01f;  // Yaw: trust SLAM

        msg.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN;
        std::fill(std::begin(msg.velocity), std::end(msg.velocity), NAN);
        std::fill(std::begin(msg.velocity_variance), std::end(msg.velocity_variance), NAN);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "NED pos: [%.3f, %.3f, %.3f] | NED yaw: %.2f deg",
            msg.position[0], msg.position[1], msg.position[2],
            px4_ros_com::frame_transforms::utils::quaternion::quaternion_get_yaw(ned_q) * 180.0 / M_PI
        );

        px4_odom_pub_->publish(msg);
    }

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_odom_sub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    float current_px4_ned_z_ = 0.0f;
    bool has_altitude_ = false;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TfToPx4Bridge>());
    rclcpp::shutdown();
    return 0;
}