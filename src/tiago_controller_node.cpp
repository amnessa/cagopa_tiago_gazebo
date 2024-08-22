#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

using namespace std::placeholders;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

// ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True [arm_type:=no-arm]
class TrajectoryActionClient : public rclcpp::Node
{
public:

    TrajectoryActionClient() : Node("trajectory_action_client")
    {
        move7={};
        move2={};
        move3={};
        move4={};
        move5={};
        move6={};
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
        cago_initial={1.565, 0.939, -0.056, 0.975, 1.738, 0.023, -0.034};  //{-1.1470 0.3418 -2.1480}
        // cago_reference1={2.228, 0.000,0.000,0.000, 0.000, 0.000, 0.000};  {-0.2391, 0.3734 ,-0.04949} left
        // cago_reference2={0.817, 0.000,0.000,0.000, 0.000, 0.000, 0.000}; {0.4615, 0.4492, 0.4492} right
        // cago_reference3={1.374, 0.598, 0.000,0.000, 0.000, 0.000, 0.000};  {0.1419, 0.3628, -0.2444} top
        move2={ -0.8046 ,   2.7079  ,  1.1632  , -2.2441   , 2.6830    ,     0, 0.000}; //0.09184,0.5204,1.186 square starts here
        move3={ 0.2055  ,  1.9221  , -0.4368 ,  -2.4438  ,  2.6931    ,     0 ,0.000}; //0.09184,0.398,0.08492
        move4={ 0.1460  ,  2.8171  ,  0.6982  , -0.5330  ,  0.8384    ,     0,0.000}; //0.2143,0.398,0.05292
        move5={  -0.2163  ,  1.6871   , 0.3538   , 0.9646  , -0.8456     ,    0,0.000};//-0.02041,-0.06122,-0.07396
        //move6={1.9253  ,  1.1397  ,  0.2383   , 1.0698  ,  1.9624  , -0.2498, -0.034}; //-1.429,-0.6122,-2.485
        //move7={1.5532  ,  1.0298  , -0.2537  ,  1.1005   ,1.8147  ,  0.2033,-0.034}; //-0.6122,0.2041,-2.188

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

        //(reference) T6 = [0.9897   0.0234   0.1409  -1.1470; -0.1401 -0.0323  0.9896  0.3418;  0.0277 -0.9992  -0.0287  -3.1480;   0    0  0  1.0000]
        // T6 =[      
        //     0.9897    0.0234    0.1409   -1.1470;
        //    -0.1401   -0.0323    0.9896    0.3418;
        //     0.0277   -0.9992   -0.0287   -3.1480;
        //          0         0         0    1.0000]
        //ros2 action send_goal /play_motion2 play_motion2_msgs/action/PlayMotion2 "{motion_name: home, skip_planning: false}"
        trajectory_msgs::msg::JointTrajectoryPoint p1,p2,p3,p4,p5,p6,p7;
        
        p1.positions = cago_initial;
        p1.time_from_start = rclcpp::Duration::from_seconds(2.0);
        points.push_back(p1);

        p2.positions= move2;
        p2.time_from_start = rclcpp::Duration::from_seconds(4.0);
        points.push_back(p2);

        p3.positions=move3;
        p3.time_from_start=rclcpp::Duration::from_seconds(6.0);
        points.push_back(p3);

        p4.positions=move4;
        p4.time_from_start=rclcpp::Duration::from_seconds(8.0);
        points.push_back(p4);

        p5.positions=move5;
        p5.time_from_start=rclcpp::Duration::from_seconds(10.0);
        points.push_back(p5);

        p2.positions= move2;
        p2.time_from_start = rclcpp::Duration::from_seconds(12.0);
        points.push_back(p2);

        // p6.positions=move6;
        // p6.time_from_start = rclcpp::Duration::from_seconds(10.0);
        // points.push_back(p6);

        // p7.positions=move7;
        // p7.time_from_start = rclcpp::Duration::from_seconds(12.0);
        // points.push_back(p7);

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
    std::vector<double> move2,move3,move4,move5,move6,move7;
    

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