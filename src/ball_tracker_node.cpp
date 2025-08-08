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
private:
    // Define the synchronization policy
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo> SyncPolicy;

public:
    BallTrackerNode() : Node("ball_tracker_node")
    {
        // Publisher for the 3D position of the ball
        publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/ball_position", 10);
        // Publisher for the debug image with bounding box
        debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/ball_tracker/debug_image", 10);

        // Subscribers for image, depth, and camera info using message filters
        image_sub_.subscribe(this, "/rsd455_img");
        depth_sub_.subscribe(this, "/rsd455_depth");
        cam_info_sub_.subscribe(this, "/rsd455_img/camera_info"); // Often published here by camera drivers

        // Create a synchronizer with a queue size of 10
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), image_sub_, depth_sub_, cam_info_sub_);

        // Set the allowable time difference (slop) between messages in seconds.
        // Let's increase this to be more tolerant of timestamp mismatches.
        sync_->setAgePenalty(0.2); // Increased to 0.2 seconds

        sync_->registerCallback(std::bind(&BallTrackerNode::synced_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        RCLCPP_INFO(this->get_logger(), "Ball tracker node started and subscribers are set up.");
    }

private:
    void synced_callback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
                         const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
                         const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg)
    {
        RCLCPP_DEBUG(this->get_logger(), "Synced callback triggered!");
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

        // Red color can wrap around in HSV, so we check two ranges
        cv::Scalar lower_red1(0, 120, 70);
        cv::Scalar upper_red1(10, 255, 255);
        cv::Scalar lower_red2(170, 120, 70);
        cv::Scalar upper_red2(180, 255, 255);
        cv::Mat mask1, mask2, mask;
        cv::inRange(hsv_image, lower_red1, upper_red1, mask1);
        cv::inRange(hsv_image, lower_red2, upper_red2, mask2);
        mask = mask1 | mask2;

        // --- Optional: Add morphological operations to clean up the mask ---
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

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

                // --- Draw bounding box and center for visualization ---
                cv::Rect bounding_box = cv::boundingRect(*largest_contour);
                cv::rectangle(cv_ptr->image, bounding_box, cv::Scalar(0, 255, 0), 2); // Green box
                cv::circle(cv_ptr->image, center, 5, cv::Scalar(0, 0, 255), -1);      // Red circle at center

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
        // Publish the debug image regardless of whether a ball was found
        debug_image_pub_->publish(*cv_ptr->toImageMsg());
        RCLCPP_DEBUG(this->get_logger(), "Publishing debug image.");
    }

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;

    // Message filters and synchronizer
    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> cam_info_sub_;

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
