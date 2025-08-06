#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <chrono>

using namespace std::chrono_literals;

class TestJointStatePublisher : public rclcpp::Node
{
public:
    TestJointStatePublisher() : Node("test_joint_state_publisher")
    {
        // Create publisher for isaac joint states
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/isaac_joint_states", 10);

        // Create timer to publish joint states at 50Hz
        timer_ = this->create_wall_timer(
            20ms, std::bind(&TestJointStatePublisher::publishJointStates, this));

        RCLCPP_INFO(this->get_logger(), "Test Joint State Publisher started");
    }

private:
    void publishJointStates()
    {
        sensor_msgs::msg::JointState joint_state_msg;
        joint_state_msg.header.stamp = this->get_clock()->now();
        joint_state_msg.header.frame_id = "";

        // UR10 arm joints
        joint_state_msg.name = {
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
            // Gripper joints
            "finger_joint",
            "right_outer_knuckle_joint",
            "left_outer_finger_joint",
            "right_outer_finger_joint",
            "left_inner_finger_joint",
            "right_inner_finger_joint",
            "left_inner_finger_pad_joint",
            "right_inner_finger_pad_joint"
        };

        // Initialize positions (simulate some movement)
        static double time = 0.0;
        time += 0.02; // 20ms increment

        joint_state_msg.position = {
            0.1 * sin(time),           // shoulder_pan_joint
            -0.5 + 0.1 * cos(time),   // shoulder_lift_joint
            1.0 + 0.1 * sin(time),    // elbow_joint
            -0.5 + 0.1 * cos(time),   // wrist_1_joint
            0.0,                      // wrist_2_joint
            0.0,                      // wrist_3_joint
            // Gripper positions
            0.04, 0.04, 0.0, 0.0, -0.04, -0.04, 0.04, 0.04
        };

        // Initialize velocities to zero
        joint_state_msg.velocity.resize(joint_state_msg.name.size(), 0.0);

        // Initialize efforts to zero
        joint_state_msg.effort.resize(joint_state_msg.name.size(), 0.0);

        joint_state_publisher_->publish(joint_state_msg);

        static int count = 0;
        if (++count % 250 == 0) { // Log every 5 seconds
            RCLCPP_INFO(this->get_logger(), "Published joint states for %zu joints",
                       joint_state_msg.name.size());
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestJointStatePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
