#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <chrono>
#include <vector>
#include <cmath>
#include <algorithm>
#include <map>

// This node no longer needs MoveIt or Eigen
// #include <moveit/robot_model_loader/robot_model_loader.h>
// #include <moveit/robot_state/robot_state.h>
// #include <moveit/robot_state/conversions.h>
// #include <Eigen/Dense>


using namespace std::chrono_literals;

// Let's use a structure closer to your original plan.
// This node will generate a target end-effector velocity.
class UR10EndEffectorControllerNode : public rclcpp::Node
{
public:
    UR10EndEffectorControllerNode() : Node("ur10_end_effector_controller_node"), time_(0.0)
    {
        end_effector_velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/end_effector_velocity", 10);
        joint_velocity_command_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/isaac_joint_commands", 10);
        joint_velocity_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/joint_velocities", 10, std::bind(&UR10EndEffectorControllerNode::joint_velocity_callback, this, std::placeholders::_1));
        target_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/target_pixel_coords", 10, std::bind(&UR10EndEffectorControllerNode::target_callback, this, std::placeholders::_1));

        // Parameters
        image_width_  = this->declare_parameter<int>("image_width", 640);
        image_height_ = this->declare_parameter<int>("image_height", 480);
        fx_ = this->declare_parameter<double>("fx", 600.0);
        fy_ = this->declare_parameter<double>("fy", 600.0);
        depth_default_ = this->declare_parameter<double>("depth_default", 0.8);
        depth_target_  = this->declare_parameter<double>("depth_target", 0.8);
        k_pixel_ = this->declare_parameter<double>("k_pixel_gain", 0.6);
        k_depth_ = this->declare_parameter<double>("k_depth_gain", 0.5);
        timeout_sec_ = this->declare_parameter<double>("lost_timeout", 0.5);
        min_manipulability_ = this->declare_parameter<double>("min_manipulability", 0.02);

        control_timer_ = this->create_wall_timer(
            100ms, std::bind(&UR10EndEffectorControllerNode::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "UR10 End-Effector Controller started (servoing enabled).");
    }

private:
    enum class Mode { SEARCHING, TRACKING };

    // This callback receives the calculated joint velocities from the jacobian node
    // and forwards them to the robot controller.
    void joint_velocity_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        joint_velocity_command_pub_->publish(*msg);
    }

    void target_callback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        last_target_px_ = *msg;
        last_target_time_ = this->now();
        have_target_ = true;
    }

    void control_loop()
    {
        // Decide mode
        auto now = this->now();
        Mode mode = Mode::SEARCHING;
        if (have_target_ && (now - last_target_time_).seconds() <= timeout_sec_)
            mode = Mode::TRACKING;

        geometry_msgs::msg::Twist twist;

        if (mode == Mode::SEARCHING)
        {
            generate_search_motion(twist);
        }
        else
        {
            generate_tracking_twist(twist);
        }

        end_effector_velocity_pub_->publish(twist);
        time_ += 0.1;
    }

    void generate_search_motion(geometry_msgs::msg::Twist & twist)
    {
        double radius = 0.05;
        double speed = 0.4;
        twist.linear.x = radius * -std::sin(time_ * speed);
        twist.linear.y = radius *  std::cos(time_ * speed);
        twist.linear.z = 0.0;
        twist.angular.x = twist.angular.y = twist.angular.z = 0.0;
    }

    void generate_tracking_twist(geometry_msgs::msg::Twist & twist)
    {
        // Image center
        double u0 = image_width_  * 0.5;
        double v0 = image_height_ * 0.5;

        double u = last_target_px_.x;
        double v = last_target_px_.y;
        double Z = (last_target_px_.z > 0.05) ? last_target_px_.z : depth_default_;

        double eu = (u - u0);
        double ev = (v - v0);

        // Simple IBVS proportional mapping (linear velocities only)
        twist.linear.x = -k_pixel_ * (eu / fx_) * Z;
        twist.linear.y = -k_pixel_ * (ev / fy_) * Z;
        twist.linear.z =  k_depth_ * (Z - depth_target_);

        // No rotation yet
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = 0.0;

        // Manipulability scaling (optional external estimate fed later).
        // For now we rely on Jacobian node logging; if you wish you can add a subscription
        // to a manipulability topic, then scale here when too low.
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr end_effector_velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_velocity_command_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_velocity_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    geometry_msgs::msg::Point last_target_px_;
    rclcpp::Time last_target_time_;
    bool have_target_{false};

    int image_width_;
    int image_height_;
    double fx_, fy_;
    double depth_default_, depth_target_;
    double k_pixel_, k_depth_;
    double timeout_sec_;
    double min_manipulability_;

    double time_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UR10EndEffectorControllerNode>();
    RCLCPP_INFO(node->get_logger(), "UR10 End-Effector Controller spinning...");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
