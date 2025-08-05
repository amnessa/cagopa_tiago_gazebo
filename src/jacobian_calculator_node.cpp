#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Dense>

class JacobianCalculatorNode : public rclcpp::Node
{
public:
    JacobianCalculatorNode() : Node("jacobian_calculator_node")
    {
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&JacobianCalculatorNode::joint_state_callback, this, std::placeholders::_1));

        joint_velocity_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/joint_velocities", 10);

        // This is a placeholder for a service that would take a desired end-effector velocity
        // and return the required joint velocities. For simplicity in this example, we will
        // directly subscribe to a velocity command.
        end_effector_velocity_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/end_effector_velocity", 10, std::bind(&JacobianCalculatorNode::velocity_callback, this, std::placeholders::_1));
    }

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        current_joint_positions_ = msg->position;
    }

    void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (current_joint_positions_.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No joint states received yet.");
            return;
        }

        Eigen::Matrix<double, 6, 6> jacobian = calculate_jacobian(current_joint_positions_);
        Eigen::Matrix<double, 6, 1> end_effector_velocity;
        end_effector_velocity << msg->linear.x, msg->linear.y, msg->linear.z,
                                 msg->angular.x, msg->angular.y, msg->angular.z;

        Eigen::Matrix<double, 6, 1> joint_velocities = jacobian.completeOrthogonalDecomposition().pseudoInverse() * end_effector_velocity;

        std_msgs::msg::Float64MultiArray joint_velocity_msg;
        joint_velocity_msg.data.assign(joint_velocities.data(), joint_velocities.data() + joint_velocities.size());
        joint_velocity_pub_->publish(joint_velocity_msg);
    }

    Eigen::Matrix<double, 6, 6> calculate_jacobian(const std::vector<double>& joint_positions)
    {
        // This is a placeholder for the actual Jacobian calculation.
        // For a real UR10, you would use a library like MoveIt! or a kinematics library
        // to get the Jacobian.
        Eigen::Matrix<double, 6, 6> J = Eigen::Matrix<double, 6, 6>::Identity();
        return J;
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr end_effector_velocity_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_velocity_pub_;
    std::vector<double> current_joint_positions_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JacobianCalculatorNode>());
    rclcpp::shutdown();
    return 0;
}
