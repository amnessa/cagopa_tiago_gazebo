#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <termios.h>
#include <unistd.h>

int getch()
{
  static struct termios oldt, newt;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  int c = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return c;
}

class MobileBaseController : public rclcpp::Node
{
public:
    MobileBaseController() : Node("mobile_base_controller")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // This is a simplified keyboard controller. For a more robust solution,
        // use the teleop_twist_keyboard package.
        std::thread{std::bind(&MobileBaseController::publish_twist, this)}.detach();
    }

private:
    void publish_twist()
    {
        while(rclcpp::ok())
        {
            int c = getch();
            geometry_msgs::msg::Twist twist;
            switch(c)
            {
                case 'w':
                    twist.linear.x = 0.5;
                    break;
                case 's':
                    twist.linear.x = -0.5;
                    break;
                case 'a':
                    twist.angular.z = 0.5;
                    break;
                case 'd':
                    twist.angular.z = -0.5;
                    break;
            }
            publisher_->publish(twist);
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MobileBaseController>());
    rclcpp::shutdown();
    return 0;
}
