#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <chrono>
#include <vector>
#include <cmath>

using namespace std::chrono_literals;

class UR10VelocityControllerNode : public rclcpp::Node
{
public:
    UR10VelocityControllerNode() : Node("ur10_velocity_controller_node"), movement_index_(0), timer_count_(0), control_mode_(0)
    {
        RCLCPP_INFO(this->get_logger(), "UR10 Velocity Controller Node starting...");

        // Initialize joint names for UR10 arm (first 6 joints)
        arm_joint_names_ = {
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        };

        // Initialize gripper joint names (remaining joints)
        gripper_joint_names_ = {
            "finger_joint",
            "right_outer_knuckle_joint",
            "left_outer_finger_joint",
            "right_outer_finger_joint",
            "left_inner_finger_joint",
            "right_inner_finger_joint",
            "left_inner_finger_pad_joint",
            "right_inner_finger_pad_joint"
        };

        // Define some predefined movements for the UR10 arm
        initializeMovements();

        // Create subscriber to joint states
        joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/isaac_joint_states", 10,
            std::bind(&UR10VelocityControllerNode::jointStateCallback, this, std::placeholders::_1));

        // Try multiple publishers for different possible command topics
        joint_command_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/isaac_joint_commands", 10);

        // Alternative publishers to try different topics
        joint_position_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/joint_position_commands", 10);

        joint_velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/joint_velocity_commands", 10);

        // Create timer for periodic movement commands
        control_timer_ = this->create_wall_timer(
            2000ms, std::bind(&UR10VelocityControllerNode::controlTimerCallback, this));

        RCLCPP_INFO(this->get_logger(), "UR10 Velocity Controller Node initialized successfully");
        RCLCPP_INFO(this->get_logger(), "Will try different command strategies:");
        RCLCPP_INFO(this->get_logger(), "  - Position commands via JointState");
        RCLCPP_INFO(this->get_logger(), "  - Velocity commands via JointState");
        RCLCPP_INFO(this->get_logger(), "  - Effort commands via JointState");
        RCLCPP_INFO(this->get_logger(), "  - Float64MultiArray position commands");
        RCLCPP_INFO(this->get_logger(), "  - Float64MultiArray velocity commands");
    }

private:
    void initializeMovements()
    {
        // Home position (all joints at 0)
        home_position_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        // Movement 1: Basic shoulder pan rotation
        movement1_ = {1.57, 0.0, 0.0, 0.0, 0.0, 0.0};  // 90 degrees shoulder pan

        // Movement 2: Lift shoulder
        movement2_ = {1.57, -0.5, 0.0, 0.0, 0.0, 0.0};

        // Movement 3: Bend elbow
        movement3_ = {1.57, -0.5, 1.2, 0.0, 0.0, 0.0};

        // Movement 4: Wrist movements
        movement4_ = {1.57, -0.5, 1.2, -0.7, 0.0, 0.0};

        // Movement 5: Full pose
        movement5_ = {0.8, -0.8, 1.5, -0.7, 1.57, 0.0};

        // Velocity commands (rad/s)
        velocity1_ = {0.5, 0.0, 0.0, 0.0, 0.0, 0.0};    // Slow shoulder pan
        velocity2_ = {0.0, 0.3, 0.0, 0.0, 0.0, 0.0};    // Slow shoulder lift
        velocity3_ = {0.0, 0.0, 0.4, 0.0, 0.0, 0.0};    // Slow elbow
        velocity4_ = {0.0, 0.0, 0.0, 0.3, 0.0, 0.0};    // Slow wrist 1
        velocity5_ = {0.0, 0.0, 0.0, 0.0, 0.3, 0.0};    // Slow wrist 2

        // Gripper open/closed positions
        gripper_open_ = {0.08, 0.08, 0.0, 0.0, -0.08, -0.08, 0.08, 0.08};
        gripper_closed_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        RCLCPP_INFO(this->get_logger(), "Initialized movements and velocities");
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (current_joint_state_.name.empty()) {
            RCLCPP_INFO(this->get_logger(), "First joint state message received. Learning joint order.");
            // Learn the joint order from the first message received
            arm_joint_names_ = msg->name;
            // Log the learned joint order
            std::string joints_log = "Learned arm joint names: ";
            for(const auto& name : arm_joint_names_) {
                joints_log += name + ", ";
            }
            RCLCPP_INFO(this->get_logger(), "%s", joints_log.c_str());
        }

        current_joint_state_ = *msg;

        // Log joint states periodically
        static int callback_count = 0;
        if (++callback_count % 50 == 0) {
            RCLCPP_INFO(this->get_logger(), "Current joint positions:");
            for (size_t i = 0; i < std::min(arm_joint_names_.size(), msg->position.size()); ++i) {
                RCLCPP_INFO(this->get_logger(), "  %s: %.4f rad",
                           arm_joint_names_[i].c_str(), msg->position[i]);
            }
        }
    }

    void controlTimerCallback()
    {
        if (current_joint_state_.name.empty()) {
            RCLCPP_WARN(this->get_logger(), "No joint states received yet, waiting...");
            return;
        }

        // Cycle through different control strategies
        switch (control_mode_) {
            case 0:
                sendPositionCommands();
                break;
            case 1:
                sendVelocityCommands();
                break;
            case 2:
                sendEffortCommands();
                break;
            case 3:
                sendFloat64PositionCommands();
                break;
            case 4:
                sendFloat64VelocityCommands();
                break;
            default:
                sendPositionCommands();
                break;
        }

        timer_count_++;

        // Change control mode every 10 attempts (20 seconds)
        if (timer_count_ % 10 == 0) {
            control_mode_ = (control_mode_ + 1) % 5;
            RCLCPP_INFO(this->get_logger(), "Switching to control mode %d", control_mode_);
        }

        // Change movement every 5 attempts (10 seconds)
        if (timer_count_ % 5 == 0) {
            movement_index_ = (movement_index_ + 1) % 5;
        }
    }

    void sendPositionCommands()
    {
        if (current_joint_state_.name.empty()) {
            RCLCPP_WARN(this->get_logger(), "Cannot send command, joint order not yet learned.");
            return;
        }
        sensor_msgs::msg::JointState command_msg;
        command_msg.header.stamp = this->get_clock()->now();
        command_msg.name = arm_joint_names_;
        // command_msg.name.insert(command_msg.name.end(), gripper_joint_names_.begin(), gripper_joint_names_.end());

        std::vector<double> target_arm_position = getCurrentTargetPosition();
        // std::vector<double> target_gripper_position = getCurrentGripperPosition();

        command_msg.position = target_arm_position;
        // command_msg.position.insert(command_msg.position.end(),
        //                            target_gripper_position.begin(), target_gripper_position.end());

        // Clear velocity and effort arrays for position control
        command_msg.velocity.clear();
        command_msg.effort.clear();

        joint_command_publisher_->publish(command_msg);
        RCLCPP_INFO(this->get_logger(), "Sent ARM POSITION command (mode 0), movement %d", movement_index_ + 1);
    }

    void sendVelocityCommands()
    {
        sensor_msgs::msg::JointState command_msg;
        command_msg.header.stamp = this->get_clock()->now();
        command_msg.name = arm_joint_names_;
        // command_msg.name.insert(command_msg.name.end(), gripper_joint_names_.begin(), gripper_joint_names_.end());

        std::vector<double> target_velocity = getCurrentTargetVelocity();
        // std::vector<double> gripper_velocity(gripper_joint_names_.size(), 0.0);

        // Clear position array for velocity control
        command_msg.position.clear();

        command_msg.velocity = target_velocity;
        // command_msg.velocity.insert(command_msg.velocity.end(),
        //                            gripper_velocity.begin(), gripper_velocity.end());

        // Clear effort array
        command_msg.effort.clear();

        joint_command_publisher_->publish(command_msg);
        RCLCPP_INFO(this->get_logger(), "Sent ARM VELOCITY command (mode 1), movement %d", movement_index_ + 1);
    }

    void sendEffortCommands()
    {
        sensor_msgs::msg::JointState command_msg;
        command_msg.header.stamp = this->get_clock()->now();
        command_msg.name = arm_joint_names_;
        // command_msg.name.insert(command_msg.name.end(), gripper_joint_names_.begin(), gripper_joint_names_.end());

        // Simple effort commands (small torques)
        std::vector<double> effort_commands = {1.0, 0.5, 0.3, 0.2, 0.1, 0.1};
        // std::vector<double> gripper_effort(gripper_joint_names_.size(), 0.0);

        // Clear position and velocity arrays
        command_msg.position.clear();
        command_msg.velocity.clear();

        command_msg.effort = effort_commands;
        // command_msg.effort.insert(command_msg.effort.end(),
        //                         gripper_effort.begin(), gripper_effort.end());

        joint_command_publisher_->publish(command_msg);
        RCLCPP_INFO(this->get_logger(), "Sent ARM EFFORT command (mode 2), movement %d", movement_index_ + 1);
    }

    void sendFloat64PositionCommands()
    {
        std_msgs::msg::Float64MultiArray position_msg;
        std::vector<double> target_position = getCurrentTargetPosition();

        position_msg.data = target_position;
        joint_position_publisher_->publish(position_msg);
        RCLCPP_INFO(this->get_logger(), "Sent FLOAT64 POSITION command (mode 3), movement %d", movement_index_ + 1);
    }

    void sendFloat64VelocityCommands()
    {
        std_msgs::msg::Float64MultiArray velocity_msg;
        std::vector<double> target_velocity = getCurrentTargetVelocity();

        velocity_msg.data = target_velocity;
        joint_velocity_publisher_->publish(velocity_msg);
        RCLCPP_INFO(this->get_logger(), "Sent FLOAT64 VELOCITY command (mode 4), movement %d", movement_index_ + 1);
    }

    std::vector<double> getCurrentTargetPosition()
    {
        switch (movement_index_) {
            case 0: return home_position_;
            case 1: return movement1_;
            case 2: return movement2_;
            case 3: return movement3_;
            case 4: return movement4_;
            default: return home_position_;
        }
    }

    std::vector<double> getCurrentTargetVelocity()
    {
        switch (movement_index_) {
            case 0: return velocity1_;
            case 1: return velocity2_;
            case 2: return velocity3_;
            case 3: return velocity4_;
            case 4: return velocity5_;
            default: return std::vector<double>(6, 0.0);
        }
    }

    std::vector<double> getCurrentGripperPosition()
    {
        // Alternate between open and closed gripper
        if ((timer_count_ / 3) % 2 == 0) {
            return gripper_open_;
        } else {
            return gripper_closed_;
        }
    }

    // Member variables
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_position_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_velocity_publisher_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    std::vector<std::string> arm_joint_names_;
    std::vector<std::string> gripper_joint_names_;

    sensor_msgs::msg::JointState current_joint_state_;

    // Predefined movements
    std::vector<double> home_position_;
    std::vector<double> movement1_, movement2_, movement3_, movement4_, movement5_;
    std::vector<double> velocity1_, velocity2_, velocity3_, velocity4_, velocity5_;
    std::vector<double> gripper_open_;
    std::vector<double> gripper_closed_;

    int movement_index_;
    int timer_count_;
    int control_mode_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<UR10VelocityControllerNode>();

    RCLCPP_INFO(node->get_logger(), "UR10 Velocity Controller Node spinning...");

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
