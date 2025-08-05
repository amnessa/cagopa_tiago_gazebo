#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <chrono>
#include <vector>
#include <cmath>
#include <algorithm> // Required for std::find
#include <map>       // Required for std::map

using namespace std::chrono_literals;
//BIG COMMENTED SECTION PLANNED TO BE ADDED
// #include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/point_stamped.hpp>
// #include <geometry_msgs/msg/twist.hpp>
// #include <std_msgs/msg/float64_multi_array.hpp>
// #include <nav_msgs/msg/odometry.hpp>

// class UR10ArmControllerNode : public rclcpp::Node
// {
// public:
//     UR10ArmControllerNode() : Node("ur10_arm_controller_node")
//     {
//         ball_position_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
//             "/ball_position", 10, std::bind(&UR10ArmControllerNode::ball_position_callback, this, std::placeholders::_1));

//         odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
//             "/odom", 10, std::bind(&UR10ArmControllerNode::odom_callback, this, std::placeholders::_1));

//         end_effector_velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/end_effector_velocity", 10);

//         joint_velocity_command_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/joint_velocity_controller/commands", 10);

//         joint_velocity_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
//             "/joint_velocities", 10, std::bind(&UR10ArmControllerNode::joint_velocity_callback, this, std::placeholders::_1));
//     }

// private:
//     void ball_position_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
//     {
//         geometry_msgs::msg::Twist twist_msg;
//         // Proportional controller to move the end-effector towards the ball
//         twist_msg.linear.x = 0.5 * msg->point.x;
//         twist_msg.linear.y = 0.5 * msg->point.y;
//         twist_msg.linear.z = 0.5 * (1.0 - msg->point.z); // Try to maintain a 1m distance
//         end_effector_velocity_pub_->publish(twist_msg);
//     }

//     void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
//     {
//         // This is where you would incorporate the mobile base's movement
//         // to adjust the arm's target. For now, we'll just log it.
//         RCLCPP_INFO(this->get_logger(), "Odometry received");
//     }

//     void joint_velocity_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
//     {
//         // Directly pass through the calculated joint velocities to the robot controller
//         joint_velocity_command_pub_->publish(*msg);
//     }

//     rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr ball_position_sub_;
//     rclcpp::Subscription<nav_msgs/msg::Odometry>::SharedPtr odom_sub_;
//     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr end_effector_velocity_pub_;
//     rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_velocity_command_pub_;
//     rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_velocity_sub_;
// };

// int main(int argc, char * argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<UR10ArmControllerNode>());
//     rclcpp::shutdown();
//     return 0;
// }


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
        velocity2_ = {0.0, 0.5, 0.0, 0.0, 0.0, 0.0};    // Slow shoulder lift
        velocity3_ = {0.0, 0.0, 0.5, 0.0, 0.0, 0.0};    // Slow elbow
        velocity4_ = {0.0, 0.0, 0.0, 0.5, 0.0, 0.0};    // Slow wrist 1
        velocity5_ = {0.0, 0.0, 0.0, 0.0, 0.5, 0.0};    // Slow wrist 2

        // Gripper open/closed positions
        gripper_open_ = {0.08, 0.08, 0.0, 0.0, -0.08, -0.08, 0.08, 0.08};
        gripper_closed_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        RCLCPP_INFO(this->get_logger(), "Initialized movements and velocities");
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (current_joint_state_.name.empty()) {
            RCLCPP_INFO(this->get_logger(), "First joint state message received. Learning joint order.");

            // Store the full state
            current_joint_state_ = *msg;

            // Filter and reorder our internal joint lists to match the received order
            std::vector<std::string> received_joints = msg->name;
            std::vector<std::string> temp_arm_joints;
            std::vector<std::string> temp_gripper_joints;

            // Create a copy of the initial lists to check against
            const std::vector<std::string> initial_arm_joints = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
            const std::vector<std::string> initial_gripper_joints = {"finger_joint", "right_outer_knuckle_joint", "left_outer_finger_joint", "right_outer_finger_joint", "left_inner_finger_joint", "right_inner_finger_joint", "left_inner_finger_pad_joint", "right_inner_finger_pad_joint"};

            for (const auto& received_joint : received_joints) {
                if (std::find(initial_arm_joints.begin(), initial_arm_joints.end(), received_joint) != initial_arm_joints.end()) {
                    temp_arm_joints.push_back(received_joint);
                } else if (std::find(initial_gripper_joints.begin(), initial_gripper_joints.end(), received_joint) != initial_gripper_joints.end()) {
                    temp_gripper_joints.push_back(received_joint);
                }
            }

            arm_joint_names_ = temp_arm_joints;
            gripper_joint_names_ = temp_gripper_joints;

            // Create a mapping from the initial hardcoded order to the learned order
            arm_joint_map_.resize(initial_arm_joints.size());
            for(size_t i = 0; i < initial_arm_joints.size(); ++i) {
                auto it = std::find(arm_joint_names_.begin(), arm_joint_names_.end(), initial_arm_joints[i]);
                if (it != arm_joint_names_.end()) {
                    arm_joint_map_[i] = std::distance(arm_joint_names_.begin(), it);
                } else {
                    // Handle error: a joint defined in the initial list was not found in the received list
                    RCLCPP_ERROR(this->get_logger(), "Could not find joint '%s' in received joint states!", initial_arm_joints[i].c_str());
                }
            }

            // Log the learned joint order
            std::string arm_log = "Learned and ordered ARM joints: ";
            for(const auto& name : arm_joint_names_) { arm_log += name + ", "; }
            RCLCPP_INFO(this->get_logger(), "%s", arm_log.c_str());

            std::string gripper_log = "Learned and ordered GRIPPER joints: ";
            for(const auto& name : gripper_joint_names_) { gripper_log += name + ", "; }
            RCLCPP_INFO(this->get_logger(), "%s", gripper_log.c_str());
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

    std::vector<double> reorder_vector(const std::vector<double>& in_vec)
    {
        if (arm_joint_map_.empty()) return in_vec; // Return original if map is not ready
        std::vector<double> out_vec(in_vec.size());
        for(size_t i = 0; i < in_vec.size(); ++i) {
            out_vec[arm_joint_map_[i]] = in_vec[i];
        }
        return out_vec;
    }

    std::vector<double> getCurrentTargetPosition()
    {
        std::vector<double> target_position;
        switch (movement_index_) {
            case 0: target_position = home_position_; break;
            case 1: target_position = movement1_; break;
            case 2: target_position = movement2_; break;
            case 3: target_position = movement3_; break;
            case 4: target_position = movement4_; break;
            default: target_position = home_position_; break;
        }
        return reorder_vector(target_position);
    }

    std::vector<double> getCurrentTargetVelocity()
    {
        std::vector<double> target_velocity;
        switch (movement_index_) {
            case 0: target_velocity = velocity1_; break;
            case 1: target_velocity = velocity2_; break;
            case 2: target_velocity = velocity3_; break;
            case 3: target_velocity = velocity4_; break;
            case 4: target_velocity = velocity5_; break;
            default: target_velocity = std::vector<double>(6, 0.0); break;
        }
        return reorder_vector(target_velocity);
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
    std::vector<int> arm_joint_map_; // Map from initial order to learned order

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
