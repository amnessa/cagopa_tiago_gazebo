#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <chrono>
#include <vector>
#include <cmath>

using namespace std::chrono_literals;

class UR10ControllerNode : public rclcpp::Node
{
public:
    UR10ControllerNode() : Node("ur10_controller_node"), movement_index_(0), timer_count_(0)
    {
        RCLCPP_INFO(this->get_logger(), "UR10 Controller Node starting...");

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
            std::bind(&UR10ControllerNode::jointStateCallback, this, std::placeholders::_1));

        // Create publisher for joint commands
        joint_command_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/isaac_joint_commands", 10);

        // Create timer for periodic movement commands
        control_timer_ = this->create_wall_timer(
            3000ms, std::bind(&UR10ControllerNode::controlTimerCallback, this));

        RCLCPP_INFO(this->get_logger(), "UR10 Controller Node initialized successfully");
        RCLCPP_INFO(this->get_logger(), "Listening to /isaac_joint_states");
        RCLCPP_INFO(this->get_logger(), "Publishing to /isaac_joint_commands");
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

        // Gripper open position (approximate values)
        gripper_open_ = {0.08, 0.08, 0.0, 0.0, -0.08, -0.08, 0.08, 0.08};

        // Gripper closed position (approximate values)
        gripper_closed_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        RCLCPP_INFO(this->get_logger(), "Initialized %zu predefined movements", 6);
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        current_joint_state_ = *msg;

        // Log joint states periodically (every 100 messages to avoid spam)
        static int callback_count = 0;
        if (++callback_count % 100 == 0) {
            RCLCPP_INFO(this->get_logger(), "Received joint states with %zu joints", msg->name.size());

            // Print arm joint positions
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

        // Create command message
        sensor_msgs::msg::JointState command_msg;
        command_msg.header.stamp = this->get_clock()->now();

        // Set all joint names (arm + gripper)
        command_msg.name = arm_joint_names_;
        command_msg.name.insert(command_msg.name.end(), gripper_joint_names_.begin(), gripper_joint_names_.end());

        // Get current movement based on timer count
        std::vector<double> target_arm_position = getCurrentTargetPosition();
        std::vector<double> target_gripper_position = getCurrentGripperPosition();

        // Combine arm and gripper positions
        command_msg.position = target_arm_position;
        command_msg.position.insert(command_msg.position.end(),
                                   target_gripper_position.begin(), target_gripper_position.end());

        // Set velocities to zero (position control)
        command_msg.velocity.resize(command_msg.position.size(), 0.0);

        // Set efforts to zero (let controller handle)
        command_msg.effort.resize(command_msg.position.size(), 0.0);

        // Publish command
        joint_command_publisher_->publish(command_msg);

        RCLCPP_INFO(this->get_logger(), "Sent movement %d to UR10", movement_index_ + 1);

        // Move to next movement
        timer_count_++;
        if (timer_count_ % 2 == 0) {  // Change movement every 6 seconds
            movement_index_ = (movement_index_ + 1) % 6;
        }
    }

    std::vector<double> getCurrentTargetPosition()
    {
        switch (movement_index_) {
            case 0: return home_position_;
            case 1: return movement1_;
            case 2: return movement2_;
            case 3: return movement3_;
            case 4: return movement4_;
            case 5: return movement5_;
            default: return home_position_;
        }
    }

    std::vector<double> getCurrentGripperPosition()
    {
        // Alternate between open and closed gripper
        if ((timer_count_ / 2) % 2 == 0) {
            return gripper_open_;
        } else {
            return gripper_closed_;
        }
    }

    // Member variables
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_publisher_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    std::vector<std::string> arm_joint_names_;
    std::vector<std::string> gripper_joint_names_;

    sensor_msgs::msg::JointState current_joint_state_;

    // Predefined movements
    std::vector<double> home_position_;
    std::vector<double> movement1_;
    std::vector<double> movement2_;
    std::vector<double> movement3_;
    std::vector<double> movement4_;
    std::vector<double> movement5_;
    std::vector<double> gripper_open_;
    std::vector<double> gripper_closed_;

    int movement_index_;
    int timer_count_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<UR10ControllerNode>();

    RCLCPP_INFO(node->get_logger(), "UR10 Controller Node spinning...");

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
