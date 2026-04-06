#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <px4_ros_com/frame_transforms.h>

using namespace std::placeholders;
using px4_msgs::msg::VehicleOdometry;

class OdomBridge : public rclcpp::Node
{
public:
    OdomBridge() : Node("px4_odom_bridge")
    {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        sub_ = this->create_subscription<VehicleOdometry>(
            "/fmu/out/vehicle_odometry",
            qos,
            std::bind(&OdomBridge::callback, this, _1));

        pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    }

private:
    void callback(const VehicleOdometry::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "Got PX4 odom");
        using namespace px4_ros_com::frame_transforms;

        // Position
        Eigen::Vector3d pos_ned(msg->position[0], msg->position[1], msg->position[2]);
        Eigen::Vector3d pos_enu = ned_to_enu_local_frame(pos_ned);

        // Velocity
        Eigen::Vector3d vel_ned(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
        Eigen::Vector3d vel_enu = ned_to_enu_local_frame(vel_ned);

        // Orientation
        Eigen::Quaterniond q_ned(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
        Eigen::Quaterniond q_enu = px4_to_ros_orientation(q_ned);

        nav_msgs::msg::Odometry odom;

        odom.header.stamp = this->get_clock()->now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        // Pose
        odom.pose.pose.position.x = pos_enu.x();
        odom.pose.pose.position.y = pos_enu.y();
        odom.pose.pose.position.z = pos_enu.z();

        odom.pose.pose.orientation.w = q_enu.w();
        odom.pose.pose.orientation.x = q_enu.x();
        odom.pose.pose.orientation.y = q_enu.y();
        odom.pose.pose.orientation.z = q_enu.z();

        // Twist
        odom.twist.twist.linear.x = vel_enu.x();
        odom.twist.twist.linear.y = vel_enu.y();
        odom.twist.twist.linear.z = vel_enu.z();

        pub_->publish(odom);
    }

    rclcpp::Subscription<VehicleOdometry>::SharedPtr sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomBridge>());
    rclcpp::shutdown();
    return 0;
}