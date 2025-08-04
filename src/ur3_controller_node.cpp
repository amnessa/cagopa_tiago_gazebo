#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

using namespace std::placeholders;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

// UR3 robot controller node - based on Tiago controller
class UR3TrajectoryActionClient : public rclcpp::Node
{
public:

    UR3TrajectoryActionClient() : Node("ur3_trajectory_action_client")
    {
        move1={};
        move2={};
        move3={};
        move4={};
        move5={};
        move6={};
        home_position={};
        RCLCPP_INFO(this->get_logger(), "IN UR3 Constructor");
        this->ur3_control_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this,
            "/scaled_joint_trajectory_controller/follow_joint_trajectory");

        this->send_goal();
    }

private:


    void send_goal()
    {
        //wait for action server
        if (!this->ur3_control_client_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
            return;
        }

        //create the goal
        auto goal_msg = FollowJointTrajectory::Goal();
        goal_msg.trajectory.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
        std::vector<trajectory_msgs::msg::JointTrajectoryPoint> points;

        // Home position for UR3 robot (all joints at 0 or safe position)
        home_position = {0.0, -1.57, 0.0, -1.57, 0.0, 0.0};  // Home configuration

        // Predefined motion sequences for UR3
        // UR3 Joint limits (approximate):
        // shoulder_pan_joint: [-2π, 2π]
        // shoulder_lift_joint: [-2π, 2π]
        // elbow_joint: [-π, π]
        // wrist_1_joint: [-2π, 2π]
        // wrist_2_joint: [-2π, 2π]
        // wrist_3_joint: [-2π, 2π]

        move1 = {0.5, -1.2, 0.8, -1.0, 0.5, 0.2};    // First position
        move2 = {1.0, -0.8, 1.2, -0.5, 1.0, 0.5};    // Second position
        move3 = {1.5, -0.5, 0.5, -2.0, 1.5, 1.0};    // Third position
        move4 = {0.8, -1.8, -0.3, -1.5, 0.8, -0.5};  // Fourth position
        move5 = {-0.5, -1.0, 1.0, -0.8, -0.5, 0.8};  // Fifth position
        move6 = {-1.0, -1.5, 0.2, -1.2, -1.0, -0.3}; // Sixth position

        trajectory_msgs::msg::JointTrajectoryPoint p1, p2, p3, p4, p5, p6, p7;

        // Start from home position
        p1.positions = home_position;
        p1.time_from_start = rclcpp::Duration::from_seconds(2.0);
        points.push_back(p1);

        // Execute motion sequence
        p2.positions = move1;
        p2.time_from_start = rclcpp::Duration::from_seconds(5.0);
        points.push_back(p2);

        p3.positions = move2;
        p3.time_from_start = rclcpp::Duration::from_seconds(8.0);
        points.push_back(p3);

        p4.positions = move3;
        p4.time_from_start = rclcpp::Duration::from_seconds(11.0);
        points.push_back(p4);

        p5.positions = move4;
        p5.time_from_start = rclcpp::Duration::from_seconds(14.0);
        points.push_back(p5);

        p6.positions = move5;
        p6.time_from_start = rclcpp::Duration::from_seconds(17.0);
        points.push_back(p6);

        p7.positions = move6;
        p7.time_from_start = rclcpp::Duration::from_seconds(20.0);
        points.push_back(p7);

        // Return to home position
        p1.positions = home_position;
        p1.time_from_start = rclcpp::Duration::from_seconds(23.0);
        points.push_back(p1);

        goal_msg.trajectory.points = points;

        //add callbacks
        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();

        send_goal_options.result_callback =
            std::bind(&UR3TrajectoryActionClient::goal_result_callback, this, _1);
        send_goal_options.goal_response_callback =
            std::bind(&UR3TrajectoryActionClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&UR3TrajectoryActionClient::goal_feedback_callback, this, _1, _2);

        //send the goal
        RCLCPP_INFO(this->get_logger(),"Sending UR3 trajectory goal");
        this->ur3_control_client_->async_send_goal(goal_msg, send_goal_options);
    }

    //callback to know if the goal was accepted or rejected
    void goal_response_callback(const GoalHandleFollowJointTrajectory::SharedPtr &goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "UR3 Goal was rejected by server");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "UR3 Goal accepted by server, waiting for result");
        }
    }

    //callback to receive feedback during goal execution
    void goal_feedback_callback(const GoalHandleFollowJointTrajectory::SharedPtr &goal_handle, const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback)
    {
        (void) goal_handle;
        (void) feedback;
        //RCLCPP_INFO(this->get_logger(), "UR3 Received feedback");
    }

    void goal_result_callback(const GoalHandleFollowJointTrajectory::WrappedResult &result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "UR3 Goal succeeded");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "UR3 Goal was aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "UR3 Goal was canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "UR3 Unknown result code");
                break;
        }
        rclcpp::shutdown();
    }

    std::vector<double> home_position;
    std::vector<double> move1, move2, move3, move4, move5, move6;

    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr ur3_control_client_;
    GoalHandleFollowJointTrajectory::SharedPtr goal_handle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UR3TrajectoryActionClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
