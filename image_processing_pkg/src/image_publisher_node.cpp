#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h> // Include cv_bridge header

int main(int argc, char** argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;

    // Create image publisher
    ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("/insta360/image_get", 1);

    // Load image from file
    cv::Mat image = cv::imread("./src/image_processing_pkg/img/image.png", cv::IMREAD_COLOR);
    if (image.empty())
    {
        ROS_ERROR("Failed to load image");
        return 1;
    }

    // Create image message
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

    // Set publishing frequency
    ros::Rate rate(30);

    while (ros::ok())
    {
        // Publish image message
        image_pub.publish(msg);

        // Sleep to maintain the desired publishing frequency
        rate.sleep();
    }

    return 0;
}