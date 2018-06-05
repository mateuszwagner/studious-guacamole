#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.hpp>

void imageCallback(const sensor_msgs::ImageConstPtr& imageMsg) {
    ROS_INFO_STREAM("Image no." << imageMsg->header.seq);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_test");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/camera/image_raw", 10, imageCallback, image_transport::TransportHints("compressed"));

    ros::spin();


    return 0;
}
