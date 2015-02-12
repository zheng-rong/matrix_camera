
#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>


void imageLeftCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO("Left Image Time:  %f", msg->header.stamp.toSec());
    //std::cout << "Left Image Time: " << msg->header.stamp.toSec() << std::endl;
    return;
}

void imageRightCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO("Right Image Time: %f", msg->header.stamp.toSec());
    //std::cout << "Right Image Time: " << msg->header.stamp.toSec() << std::endl;
    return;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "image_time_reporter");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub_left  = it.subscribe("/stereo/left/image_raw",  1, imageLeftCallback);
    image_transport::Subscriber sub_right = it.subscribe("/stereo/right/image_raw", 1, imageRightCallback);


    ros::spin();

    return 0;
}
