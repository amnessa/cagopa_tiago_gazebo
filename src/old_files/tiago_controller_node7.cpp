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
        x_move={};
        z_move={};
        x_move2={};
        y_move={};
        y_move2={};
        z_move2={};
        cago_initial={};
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
        std::vector<trajectory_msgs::msg::JointTrajectoryPoint> points;

        // An initial Position for the robot tahta min mak H -> robot min 78 cm tahta min 90cm max 207cm
        cago_initial={1.565, 0.939, -0.056, 0.975, 1.738, 0.023, -0.034};
        x_move={-0.0608  ,  1.0785  , -0.3617  ,  0.9814  , -1.7359   ,-0.2863  ,  1.9721}; //plus 1
        z_move={1.7933  ,  1.8351  , -0.4234 ,   1.6007  ,  0.9507  ,  0.4831  ,  0.8766}; //negative 1
        x_move2={-0.0608  ,  1.0785  , -0.3617  ,  0.9814  , -1.7359  , -0.2863   , 1.9721}; //negative 1
        y_move={-0.8264 ,   1.0822 ,  -0.3576  ,  0.8897 ,  -1.9505 ,  -0.3266  ,  1.4360};//plus 1
        y_move2={0.2854  ,  1.2198  , -0.4036 ,   0.9391   ,-2.1732  , -0.4148  ,  2.7563}; //negative 0.5
        z_move2={1.7933   , 1.8351  , -0.4234 ,   1.6007  ,  0.9507   , 0.4831 ,   0.8766}; //negative 1
        //                  1    2    3    4    5    6    7
        // % Set joint limits for the first joint
        // robot.links(1).qlim = [-pi/2, pi/2];
        // trajectory_msgs::msg::JointTrajectoryPoint::time_from_start
        // arm_1_joint: <limit effort="43.0" lower="0.0" upper="2.748893571891069" velocity="1.95"/>
        // arm_2_joint: <limit effort="43.0" lower="-1.5707963267948966" upper="1.090830782496456" velocity="1.95"/>
        // arm_3_joint: <limit effort="26" lower="-3.5342917352885173" upper="1.5707963267948966" velocity="2.35"/>
        // arm_4_joint: <limit effort="26" lower="-0.39269908169872414" upper="2.356194490192345" velocity="2.35"/>
        // arm_5_joint: <limit effort="3" lower="-2.0943951023931953" upper="2.0943951023931953" velocity="1.95"/>
        // arm_6_joint: <limit effort="6.6" lower="-1.413716694115407" upper="1.413716694115407" velocity="1.76"/>
        // arm_7_joint: <limit effort="6.6" lower="-2.0943951023931953" upper="2.0943951023931953" velocity="1.76"/>
        // arm_tool_joint: fixed

        // T7 =[0.9940 -0.1072 0.0234 -1.147; -0.1064 -0.9938 -0.0323 0.3418; 0.0267 0.0296 -0.9992 -2.148; 0 0 0 1]
        //ros2 action send_goal /play_motion2 play_motion2_msgs/action/PlayMotion2 "{motion_name: tuck, skip_planning: false}"
        trajectory_msgs::msg::JointTrajectoryPoint p1,p2,p3,p4,p5,p6,p7;
        
        p1.positions = cago_initial;
        p1.time_from_start = rclcpp::Duration::from_seconds(2.0);
        points.push_back(p1);

        // p2.positions= x_move;
        // p2.time_from_start = rclcpp::Duration::from_seconds(6.0);
        // points.push_back(p2);

        p4.positions=x_move2;
        p4.time_from_start=rclcpp::Duration::from_seconds(4.0);
        points.push_back(p4);

        p5.positions=y_move;
        p5.time_from_start=rclcpp::Duration::from_seconds(6.0);
        points.push_back(p5);

        p6.positions=y_move2;
        p6.time_from_start=rclcpp::Duration::from_seconds(8.0);
        points.push_back(p6);

        p3.positions=z_move;
        p3.time_from_start = rclcpp::Duration::from_seconds(10.0);
        points.push_back(p3);

        p7.positions=z_move2;
        p7.time_from_start = rclcpp::Duration::from_seconds(12.0);

        p1.positions = cago_initial;
        p1.time_from_start = rclcpp::Duration::from_seconds(14.0);
        points.push_back(p1);

        //point.velocities = 

        goal_msg.trajectory.points = points;
        
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
    
    std::vector<double> cago_initial;
    std::vector<double> x_move,x_move2;
    std::vector<double> z_move,z_move2;
    std::vector<double> y_move,y_move2;

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