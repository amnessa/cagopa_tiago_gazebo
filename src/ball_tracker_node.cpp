#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/msg/camera_info.hpp>

class BallTrackerNode : public rclcpp::Node
{
public:
    BallTrackerNode() : Node("ball_tracker_node")
    {
        // Publisher for the 3D position of the ball
        publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/ball_position", 10);

        // Subscribers for image, depth, and camera info using message filters
        image_sub_.subscribe(this, "/rsd455_img");
        depth_sub_.subscribe(this, "/rsd455_depth");
        cam_info_sub_.subscribe(this, "/rsd455_img/camera_info"); // Often published here by camera drivers

        // Synchronizer to get corresponding messages
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), image_sub_, depth_sub_, cam_info_sub_);
        sync_->registerCallback(std::bind(&BallTrackerNode::synced_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    }

private:
    void synced_callback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
                         const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
                         const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg)
    {
        cv_bridge::CvImagePtr cv_ptr, depth_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
            depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // --- Ball detection in color image ---
        cv::Mat hsv_image;
        cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

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

                // --- 3D position estimation ---
                // Get depth value at the center of the ball
                float depth = depth_ptr->image.at<float>(center);

                // Use camera intrinsics to get 3D point
                cam_model_.fromCameraInfo(info_msg);
                cv::Point3d ball_3d_position = cam_model_.projectPixelTo3dRay(center) * depth;

                // Publish the 3D point
                geometry_msgs::msg::PointStamped point_msg;
                point_msg.header.stamp = this->now();
                point_msg.header.frame_id = image_msg->header.frame_id; // Use camera frame
                point_msg.point.x = ball_3d_position.x;
                point_msg.point.y = ball_3d_position.y;
                point_msg.point.z = ball_3d_position.z;
                publisher_->publish(point_msg);
            }
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;

    // Message filters and synchronizer
    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> cam_info_sub_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo> SyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    image_geometry::PinholeCameraModel cam_model_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BallTrackerNode>());
    rclcpp::shutdown();
    return 0;
}
