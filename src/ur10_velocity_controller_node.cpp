#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
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
        // This publisher sends the desired end-effector motion to the jacobian_calculator_node
        end_effector_velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/end_effector_velocity", 10);

        // The jacobian_calculator_node will publish joint velocities, which we send to the robot.
        // The topic name must match the one Isaac Sim is listening to.
        joint_velocity_command_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/isaac_joint_commands", 10);

        // We subscribe to the output of the jacobian_calculator_node
        joint_velocity_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/joint_velocities", 10, std::bind(&UR10EndEffectorControllerNode::joint_velocity_callback, this, std::placeholders::_1));

        // Timer to periodically generate a new velocity command
        control_timer_ = this->create_wall_timer(
            100ms, std::bind(&UR10EndEffectorControllerNode::generate_velocity_command, this));

        RCLCPP_INFO(this->get_logger(), "UR10 End-Effector Controller started.");
        RCLCPP_INFO(this->get_logger(), "Publishing target Twist to /end_effector_velocity.");
        RCLCPP_INFO(this->get_logger(), "Subscribing to /joint_velocities and forwarding to /joint_velocity_controller/commands.");
    }

private:
    // This callback receives the calculated joint velocities from the jacobian node
    // and forwards them to the robot controller.
    void joint_velocity_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        joint_velocity_command_pub_->publish(*msg);
    }

    // This function generates a sample command to make the end-effector move.
    // Here, it creates a circular motion.
    void generate_velocity_command()
    {
        geometry_msgs::msg::Twist twist_msg;

        double radius = 0.1; // meters
        double speed = 0.2; // rad/s

        // Calculate a simple circular trajectory in the XY plane of the end-effector frame
        twist_msg.linear.x = radius * -sin(time_ * speed);
        twist_msg.linear.y = radius * cos(time_ * speed);
        twist_msg.linear.z = 0.0; // No vertical movement

        // No rotational movement for this test
        twist_msg.angular.x = 0.0;
        twist_msg.angular.y = 0.0;
        twist_msg.angular.z = 0.0;

        end_effector_velocity_pub_->publish(twist_msg);

        time_ += 0.1; // Increment time for the next step (since timer is 100ms)
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr end_effector_velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_velocity_command_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_velocity_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
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
