#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <Eigen/Dense>
#include <memory>
#include <limits>

// MoveIt Includes
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

class JacobianCalculatorNode : public rclcpp::Node
{
public:
    JacobianCalculatorNode() : Node("jacobian_calculator_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
    {
        // Constructor is now intentionally simple.
    }

    // New init method to be called after the node is a shared_ptr
    void init()
    {
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/isaac_joint_states", 10, std::bind(&JacobianCalculatorNode::joint_state_callback, this, std::placeholders::_1));

        joint_velocity_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/joint_velocities", 10);
        joint_state_cmd_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/isaac_joint_commands", 10);

        // Parameters
        this->declare_parameter<std::string>("planning_group", "ur_manipulator");
        this->declare_parameter<std::string>("end_effector_link", "wrist_3_link");
        this->declare_parameter<double>("min_manipulability", 0.02);
        min_manipulability_ = this->get_parameter("min_manipulability").as_double();

        // Initialize MoveIt components
        RCLCPP_INFO(this->get_logger(), "Loading robot model...");
        robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(shared_from_this(), "robot_description");
        robot_model_ = robot_model_loader_->getModel();

        if (!robot_model_) {
            RCLCPP_FATAL(this->get_logger(), "Failed to load robot model. Is the robot_description parameter set?");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Robot model loaded successfully.");

        robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
        robot_state_->setToDefaultValues();

        // Get planning group and end-effector link from parameters or use defaults
        planning_group_name_ = this->get_parameter("planning_group").as_string();
        end_effector_link_name_ = this->get_parameter("end_effector_link").as_string();

        joint_model_group_ = robot_model_->getJointModelGroup(planning_group_name_);
        if (!joint_model_group_) {
            RCLCPP_FATAL(this->get_logger(), "Planning group '%s' not found.", planning_group_name_.c_str());
            rclcpp::shutdown();
        }
    }

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (robot_state_)
        {
            // Filter the incoming joint state message to only include joints known to our robot model.
            // This prevents errors if the topic contains joints for other hardware, like a gripper.
            std::vector<std::string> known_joint_names;
            std::vector<double> known_joint_positions;

            for (size_t i = 0; i < msg->name.size(); ++i)
            {
                // Check if the joint from the message exists in our model
                if (robot_state_->getRobotModel()->hasJointModel(msg->name[i]))
                {
                    known_joint_names.push_back(msg->name[i]);
                    known_joint_positions.push_back(msg->position[i]);
                }
            }

            if (!known_joint_names.empty())
            {
                robot_state_->setVariablePositions(known_joint_names, known_joint_positions);
            }
        }
    }

    void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (!robot_state_ || !joint_model_group_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Robot model not ready yet.");
            return;
        }

        Eigen::MatrixXd jacobian = calculate_jacobian();
        if (jacobian.rows() != 6)
        {
            RCLCPP_WARN(this->get_logger(), "Unexpected Jacobian size %ldx%ld", jacobian.rows(), jacobian.cols());
            return;
        }

        // Manipulability (only compute when square 6x6)
        if (jacobian.cols() == 6)
        {
            Eigen::MatrixXd JJt = jacobian * jacobian.transpose();
            double detJJt = JJt.determinant();
            if (detJJt > 0.0)
            {
                double manipulability = std::sqrt(detJJt);
                last_manipulability_ = manipulability;
                RCLCPP_DEBUG(this->get_logger(), "Manipulability: %.5f", manipulability);
            }
        }

        Eigen::Matrix<double, 6, 1> end_effector_velocity;
        end_effector_velocity << msg->linear.x, msg->linear.y, msg->linear.z,
                                 msg->angular.x, msg->angular.y, msg->angular.z;

        // Pseudoinverse
        Eigen::MatrixXd pinv = jacobian.completeOrthogonalDecomposition().pseudoInverse();
        Eigen::VectorXd joint_velocities = pinv * end_effector_velocity;

        // Publish raw array (legacy)
        std_msgs::msg::Float64MultiArray joint_velocity_msg;
        joint_velocity_msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
        joint_velocity_msg.layout.dim[0].size = joint_velocities.size();
        joint_velocity_msg.layout.dim[0].stride = 1;
        joint_velocity_msg.layout.dim[0].label = "joint_velocities";
        joint_velocity_msg.data.resize(joint_velocities.size());
        Eigen::VectorXd::Map(&joint_velocity_msg.data[0], joint_velocities.size()) = joint_velocities;
        joint_velocity_pub_->publish(joint_velocity_msg);

        // Publish JointState with velocities (Isaac expects JointState)
        sensor_msgs::msg::JointState js;
        js.header.stamp = this->now();
        const auto & names = joint_model_group_->getVariableNames();
        js.name = names;  // expects order matching robot
        js.velocity.assign(joint_velocities.data(), joint_velocities.data() + joint_velocities.size());
        // Fill position with NaN to indicate velocity control (per Isaac / doc guidance)
        js.position.resize(js.velocity.size(), std::numeric_limits<double>::quiet_NaN());
        js.effort.resize(js.velocity.size(), 0.0);
        joint_state_cmd_pub_->publish(js);
    }

    Eigen::MatrixXd calculate_jacobian()
    {
        Eigen::MatrixXd jacobian;
        // The reference point for the Jacobian is the origin of the end-effector link
        Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
        robot_state_->getJacobian(
            joint_model_group_,
            robot_state_->getLinkModel(end_effector_link_name_),
            reference_point_position,
            jacobian);
        return jacobian;
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr end_effector_velocity_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_velocity_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_cmd_pub_;
    double min_manipulability_{0.02};
    double last_manipulability_{0.0};

    // MoveIt Members
    std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
    moveit::core::RobotModelPtr robot_model_;
    moveit::core::RobotStatePtr robot_state_;
    const moveit::core::JointModelGroup* joint_model_group_;
    std::string planning_group_name_;
    std::string end_effector_link_name_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JacobianCalculatorNode>();
    node->init(); // Call init() here
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
