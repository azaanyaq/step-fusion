// ROS 2 wrapper around the StepFusion core: Imu + MagneticField in, fused/raw paths + heading out

#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/float64.hpp>

#include "stepfusion/pdr.hpp"

class StepFusionNode : public rclcpp::Node {
public:
    StepFusionNode() : Node("stepfusion_node"), pdr_(load_config()) {
        frame_id_ = declare_parameter<std::string>("frame_id", "odom");
        const std::string imu_topic = declare_parameter<std::string>("imu_topic", "/imu/data_raw");
        const std::string mag_topic = declare_parameter<std::string>("mag_topic", "/imu/mag");

        fused_path_.header.frame_id = frame_id_;
        raw_path_.header.frame_id = frame_id_;

        fused_path_pub_ = create_publisher<nav_msgs::msg::Path>("~/path", 10);
        raw_path_pub_ = create_publisher<nav_msgs::msg::Path>("~/path_raw", 10);
        heading_pub_ = create_publisher<std_msgs::msg::Float64>("~/heading", 10);

        // Best effort for high rate sensor streams
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::Imu& msg) { on_imu(msg); });
        mag_sub_ = create_subscription<sensor_msgs::msg::MagneticField>(
            mag_topic, rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::MagneticField& msg) { on_mag(msg); });

        RCLCPP_INFO(get_logger(), "listening on %s and %s", imu_topic.c_str(), mag_topic.c_str());
    }

private:
    // Called in the initialiser list, the Node base is already constructed so declare_parameter is fine
    stepfusion::PdrConfig load_config() {
        stepfusion::PdrConfig config;
        config.process_noise_q = declare_parameter<double>("process_noise_q", 0.0);
        config.measurement_noise_r = declare_parameter<double>("measurement_noise_r", 0.0);
        config.step_length_m = declare_parameter<double>("step_length_m", 0.73);
        config.step.high_threshold = declare_parameter<double>("step_high_threshold", 2.0);
        config.step.low_threshold = declare_parameter<double>("step_low_threshold", 1.3);
        config.step.min_step_gap_s = declare_parameter<double>("min_step_gap_s", 0.4);

        if (config.process_noise_q <= 0.0 || config.measurement_noise_r <= 0.0) {
            throw std::invalid_argument("process_noise_q and measurement_noise_r must be set (> 0), see config/*.yaml");
        }
        return config;
    }

    // Uses the sensor's header stamp, subtracted in integer ns to keep precision
    double seconds_since_start(const builtin_interfaces::msg::Time& stamp) {
        const int64_t ns = rclcpp::Time(stamp).nanoseconds();
        if (!first_stamp_ns_) {
            first_stamp_ns_ = ns;
        }
        return static_cast<double>(ns - *first_stamp_ns_) * 1e-9;
    }

    void on_mag(const sensor_msgs::msg::MagneticField& msg) {
        pdr_.add_mag(seconds_since_start(msg.header.stamp), msg.magnetic_field.x, msg.magnetic_field.y);
    }

    void on_imu(const sensor_msgs::msg::Imu& msg) {
        const double t = seconds_since_start(msg.header.stamp);
        const std::optional<stepfusion::StepEvent> step =
            pdr_.add_imu(t, msg.angular_velocity.z, msg.linear_acceleration.x, msg.linear_acceleration.y,
                         msg.linear_acceleration.z);

        if (!pdr_.initialised()) {
            return;
        }

        std_msgs::msg::Float64 heading;
        heading.data = pdr_.fused_heading();
        heading_pub_->publish(heading);

        if (step) {
            append_pose(fused_path_, step->fused, step->fused_heading, msg.header.stamp);
            append_pose(raw_path_, step->raw, step->raw_heading, msg.header.stamp);
            fused_path_pub_->publish(fused_path_);
            raw_path_pub_->publish(raw_path_);

            RCLCPP_INFO(get_logger(), "step %d: closing distance fused %.3f m, raw %.3f m", step->count,
                        stepfusion::closing_distance(step->fused), stepfusion::closing_distance(step->raw));
        }
    }

    void append_pose(nav_msgs::msg::Path& path, const stepfusion::Position& position, double heading,
                     const builtin_interfaces::msg::Time& stamp) {
        if (path.poses.empty()) {
            geometry_msgs::msg::PoseStamped origin;
            origin.header.frame_id = frame_id_;
            origin.header.stamp = stamp;
            origin.pose.orientation.w = 1.0;
            path.poses.push_back(origin);
        }

        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = frame_id_;
        pose.header.stamp = stamp;
        pose.pose.position.x = position.x;
        pose.pose.position.y = position.y;
        pose.pose.orientation.z = std::sin(heading / 2.0); // Yaw only quaternion
        pose.pose.orientation.w = std::cos(heading / 2.0);

        path.header.stamp = stamp;
        path.poses.push_back(pose);
    }

    stepfusion::Pdr pdr_;
    std::string frame_id_;
    std::optional<int64_t> first_stamp_ns_;

    nav_msgs::msg::Path fused_path_;
    nav_msgs::msg::Path raw_path_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr fused_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raw_path_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StepFusionNode>());
    rclcpp::shutdown();
    return 0;
}
