#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "../include/serial.h"
#include <marker_publisher/marker_pub.h>
//#include <geometry_msgs/PoseWithCovariance.h>

mtracker::Serial *com;

void markerCallback(const marker_publisher::MarkerArrayConstPtr& markerArrayMsg) {
    if (markerArrayMsg->markers.size() == 0)
        return;

    for (marker_publisher::Marker marker: markerArrayMsg->markers) {
        ROS_INFO_STREAM("Marker no." << marker.idx);
    }

    if (com->isOpen()) {
        com->setVelocities(0.0, 1.0);
        com->writeFrame();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mtracker");
    ros::NodeHandle nh;

    ros::Subscriber marker_sub = nh.subscribe("/markerpub/MarkerArray", 10, &markerCallback);
    com = new mtracker::Serial("/dev/ttyUSB0");
    com->open(921600);

    if (com->isOpen()) {
      com->setMode(MODE_SET_ODOMETRY | MODE_MOTORS_ON);
      com->setPose(0.0, 0.0, 0.0);
      com->setVelocities(0.0, 0.0);
      com->writeFrame();

      ROS_INFO("MTracker [OK]");
    }
    else {
      ROS_INFO("Could not open COM port.");
      nh.shutdown();
    }

    ros::spin();
    return 0;
}
