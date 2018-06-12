#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "../include/serial.h"
#include <marker_publisher/marker_pub.h>

mtracker::Serial com("/dev/ttyUSB0");

void markerCallback(const marker_publisher::MarkerArray& markerArrayMsg) {
    com.setVelocities(0.0, 1.0);
}

/*void imageCallback(const sensor_msgs::ImageConstPtr& imageMsg) {
    ROS_INFO_STREAM("Image no." << imageMsg->header.seq);
}*/

int main(int argc, char** argv) {
    ros::init(argc, argv, "mtracker");
    ros::NodeHandle nh;

//    image_transport::ImageTransport it(nh);
//    image_transport::Subscriber it_sub = it.subscribe("/camera/image_raw", 10, imageCallback, image_transport::TransportHints("compressed"));

    ros::Subscriber marker_sub = nh.subscribe("marker_publisher/MarkerArray", 10, &markerCallback);

    com.open(921600);

    if (com.isOpen()) {
      com.setMode(MODE_SET_ODOMETRY | MODE_MOTORS_ON);
      com.setPose(0.0, 0.0, 0.0);
      com.setVelocities(0.0, 0.0);
      com.writeFrame();

      ROS_INFO("MTracker [OK]");
    }
    else {
      ROS_INFO("Could not open COM port.");
      nh.shutdown();
    }

    ros::spin();
    return 0;
}
