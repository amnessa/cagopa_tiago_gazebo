#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "play_motion2_msgs/action/play_motion2.hpp"
//#include "play_motion2/play_motion2.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
// use msg interface to create a client. action interface does not exist in tiago
///home/cagopalin2/tiago_public_ws/install/play_motion2/include/play_motion2
//control_msgs/action/FollowJointTrajectory
// add include for example motions


//find an interface!!!!
using TiagoControl = control_msgs::action::FollowJointTrajectory;
using TiagoControlGoalHandle = rclcpp_action::ClientGoalHandle<TiagoControl>;

using namespace std::placeholders;

class TiagoControllerNode : public rclcpp::Node
{
public:
    TiagoControllerNode():Node("tiago_control_client")
    {
        tiago_control_client_ = rclcpp_action::create_client<TiagoControl>(this,"tiago_control");

    }
    void send_goal()
    {
        //wait for the action server
        tiago_control_client_->wait_for_action_server();

        //create a goal
        auto goal = TiagoControl::Goal();
        goal.trajectory = 
    }

private:

    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr tiago_control_client_;

};


int main(int argc, char ** argv)
{
    rclcpp::init(argc,argv);


    rclcpp::shutdown();

    return 0;
}