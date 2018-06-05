#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <image_transport/image_transport.h>

void imageCallback(const sensor_msgs::ImageConstPtr& imageMsg) {
    ROS_INFO_STREAM("Image no." << imageMsg->header.seq);

}

void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& cameraInfo) {

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_test");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("image_raw", 1, imageCallback);
    image_transport::Publisher pub = it.advertise("out_image", 1);

//    ros::Subscriber imageSub = nh.subscribe("image_raw", 100, &imageCallback, ros::TransportHints().unreliable());
//    ros::Subscriber cameraInfoSub = nh.subscribe("camera_info", 100, &cameraInfoCallback, ros::TransportHints().unreliable());
    ros::spin();

    return 0;
}
