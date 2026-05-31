#include <rclcpp/rclcpp.hpp>

// PX4 v1.14+ uses VehicleOdometry; for older firmware use VehicleVisualOdometry
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <px4_ros_com/frame_transforms.h>

class TfToPx4Bridge : public rclcpp::Node
{
public:
    TfToPx4Bridge() : Node("tf_to_px4_bridge")
    {
        // FIX 1: get_clock() not get_get_clock()
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        px4_odom_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
            "/fmu/in/vehicle_odometry", rclcpp::QoS(10)
                .reliability(rclcpp::ReliabilityPolicy::BestEffort)  // FIX 2: PX4 uses BestEffort QoS
                .durability(rclcpp::DurabilityPolicy::TransientLocal));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&TfToPx4Bridge::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "TF→PX4 bridge started at 20Hz");
    }

private:
    void timerCallback()
    {
        geometry_msgs::msg::TransformStamped transform_stamped;
        try {
            transform_stamped = tf_buffer_->lookupTransform(
                "map", "base_link",
                rclcpp::Time(0),
                rclcpp::Duration::from_seconds(0.05));
        }
        catch (const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "TF lookup failed (map→base_link): %s", ex.what());
            return;
        }

        // --- Position: ROS ENU → PX4 NED ---
        Eigen::Vector3d enu_pos(
            transform_stamped.transform.translation.x,
            transform_stamped.transform.translation.y,
            transform_stamped.transform.translation.z
        );
        Eigen::Vector3d ned_pos =
            px4_ros_com::frame_transforms::enu_to_ned_local_frame(enu_pos);

        // --- Orientation: ROS (baselink→ENU) → PX4 (aircraft→NED) ---
        Eigen::Quaterniond enu_q(
            transform_stamped.transform.rotation.w,
            transform_stamped.transform.rotation.x,
            transform_stamped.transform.rotation.y,
            transform_stamped.transform.rotation.z
        );
        Eigen::Quaterniond ned_q =
            px4_ros_com::frame_transforms::ros_to_px4_orientation(enu_q);

        // --- Build PX4 message ---
        auto msg = px4_msgs::msg::VehicleOdometry();

        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000ULL;
        msg.timestamp_sample =
            static_cast<uint64_t>(transform_stamped.header.stamp.sec) * 1000000ULL +
            static_cast<uint64_t>(transform_stamped.header.stamp.nanosec) / 1000ULL;

        msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;

        msg.position[0] = static_cast<float>(ned_pos.x());
        msg.position[1] = static_cast<float>(ned_pos.y());
        msg.position[2] = static_cast<float>(ned_pos.z());

        msg.q[0] = static_cast<float>(ned_q.w());
        msg.q[1] = static_cast<float>(ned_q.x());
        msg.q[2] = static_cast<float>(ned_q.y());
        msg.q[3] = static_cast<float>(ned_q.z());

        msg.position_variance[0] = 0.01f;
        msg.position_variance[1] = 0.01f;
        msg.position_variance[2] = 0.1f;   // Higher Z: 2D LiDAR has no height info

        msg.orientation_variance[0] = 0.01f;  // roll  — less certain from 2D SLAM
        msg.orientation_variance[1] = 0.01f;  // pitch — less certain from 2D SLAM
        msg.orientation_variance[2] = 0.005f; // yaw   — slam_toolbox is good at this

        msg.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN;
        std::fill(std::begin(msg.velocity), std::end(msg.velocity), NAN);
        std::fill(std::begin(msg.velocity_variance), std::end(msg.velocity_variance), NAN);

        px4_odom_pub_->publish(msg);
    }

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TfToPx4Bridge>());
    rclcpp::shutdown();
    return 0;
}