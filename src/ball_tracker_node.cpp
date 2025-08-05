#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class BallTrackerNode : public rclcpp::Node
{
public:
    BallTrackerNode() : Node("ball_tracker_node")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/ball_position", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10, std::bind(&BallTrackerNode::image_callback, this, std::placeholders::_1));
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat hsv_image;
        cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

        // Define the range for red color
        cv::Scalar lower_red = cv::Scalar(0, 100, 100);
        cv::Scalar upper_red = cv::Scalar(10, 255, 255);
        cv::Mat mask;
        cv::inRange(hsv_image, lower_red, upper_red, mask);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (!contours.empty())
        {
            auto largest_contour = std::max_element(contours.begin(), contours.end(),
                [](const auto& a, const auto& b) {
                    return cv::contourArea(a) < cv::contourArea(b);
                });

            cv::Moments M = cv::moments(*largest_contour);
            if (M.m00 > 0)
            {
                cv::Point2f center(M.m10 / M.m00, M.m01 / M.m00);

                // This is a simplified 3D position estimation. For a real application,
                // you would use depth information from a depth camera or stereo vision.
                geometry_msgs::msg::PointStamped point_msg;
                point_msg.header.stamp = this->now();
                point_msg.header.frame_id = "camera_color_optical_frame";
                point_msg.point.x = (center.x - cv_ptr->image.cols / 2.0) / 100.0; // Simplified
                point_msg.point.y = (center.y - cv_ptr->image.rows / 2.0) / 100.0; // Simplified
                point_msg.point.z = 1.0; // Assume a fixed distance
                publisher_->publish(point_msg);
            }
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BallTrackerNode>());
    rclcpp::shutdown();
    return 0;
}
