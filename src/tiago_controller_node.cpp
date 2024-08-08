#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

using namespace std::placeholders;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

// ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True [arm_type:=no-arm]
// need to find a way to feed different motions to the robot (maybe a motions library)
class TrajectoryActionClient : public rclcpp::Node
{
public:

    TrajectoryActionClient() : Node("trajectory_action_client")
    {
        RCLCPP_INFO(this->get_logger(), "IN Contstructor");
        this->tiago_control_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this,
            "/arm_controller/follow_joint_trajectory");

        this->send_goal();
    }

private:
    

    void send_goal()
    {
        //wait for action server
        if (!this->tiago_control_client_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
            return;
        }

        //create the goal
        auto goal_msg = FollowJointTrajectory::Goal();
        goal_msg.trajectory.joint_names = {"arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"};
        //                                  0.070-2.679 , -1.501-1.021 , -0.323-2.286 , -2.073-2.074 , -1.394-1.394 , -1.394-1.394 , -2.074-2.074
        trajectory_msgs::msg::JointTrajectoryPoint point;

        //                  1    2    3    4    5    6    7
        point.positions = {2.0, -1.0, -0.3, -1.0, -1.0, -1.0, -2.0};
        point.time_from_start = rclcpp::Duration::from_seconds(4.0);
        //point.velocities = 

        goal_msg.trajectory.points.push_back(point);
        
        //add callbacks
        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        
        send_goal_options.result_callback = 
            std::bind(&TrajectoryActionClient::goal_result_callback, this, _1);
        send_goal_options.goal_response_callback = 
            std::bind(&TrajectoryActionClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback = 
            std::bind(&TrajectoryActionClient::goal_feedback_callback, this, _1, _2);


        //send the goal
        RCLCPP_INFO(this->get_logger(),"Sending a goal");
        this->tiago_control_client_->async_send_goal(goal_msg, send_goal_options);
    }
    
    //callback to know if the goal was accepted or rejected
    void goal_response_callback(const GoalHandleFollowJointTrajectory::SharedPtr &goal_handle)
    {
        if (!goal_handle) 
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } 
        else 
        {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    //callback to receive feedback during goal execution
    void goal_feedback_callback(const GoalHandleFollowJointTrajectory::SharedPtr &goal_handle, const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback)
    {
        (void) goal_handle;
        (void) feedback;
        //RCLCPP_INFO(this->get_logger(), "Received feedback");
    }

    void goal_result_callback(const GoalHandleFollowJointTrajectory::WrappedResult &result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                break;
        }
        rclcpp::shutdown();
    }

    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr tiago_control_client_;
    GoalHandleFollowJointTrajectory::SharedPtr goal_handle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryActionClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}