#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.hpp>
#include "MTrackerDriver.cpp"
#include <serial/serial.h>

serial::Serial com;

void imageCallback(const sensor_msgs::ImageConstPtr& imageMsg) {
    ROS_INFO_STREAM("Image no." << imageMsg->header.seq);
}

void open() {
    std::string portNumber = "/dev/ttyUSB0";
    int baudrate = 115200;
    // port, baudrate, timeout in milliseconds
    serial::Serial com(portNumber, baudrate, serial::Timeout::simpleTimeout(1000));

    ROS_DEBUG("Is the serial port open?");
    if(com.isOpen())
      ROS_DEBUG("Yes.");
    else
      ROS_DEBUG("No.");
}

void sendVelocityFrame() {
    initFrame();
    setVelocity(0.1, 0.2);
//    com.write(tx_frame);
//    ROS_DEBUG((uint8_t *)&tx_frame);


    //                initFrame();
    //				setVelocity((float)matrixData[0], (float)matrixData[1]);
    //				plhs[0] = mxCreateNumericMatrix(1,26, mxUINT8_CLASS, mxREAL);

    //				com.Send((uint8_t *)&tx_frame, 26);

    //				outputData = (uint8_t*)mxGetData(plhs[0]);
    //				for(int i = 0; i < 26; i++)
    //				{
    //					uint8_t ch = ((uint8_t *)&tx_frame)[i];
    //					outputData[i] = ch;
    //				}
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "image_test");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/camera/image_raw", 10, imageCallback, image_transport::TransportHints("compressed"));

    ros::spin();
    open();
    sendVelocityFrame();

    return 0;
}

