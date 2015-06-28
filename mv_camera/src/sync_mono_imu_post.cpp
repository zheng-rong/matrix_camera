#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <quadrotor_msgs/GPIOTime.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
// #include <opencv2/opencv.hpp>
// #include <cv_bridge/cv_bridge.h>


using namespace std;

#define IMAGE_FRAME_TIME_TOLERANCE 2.5e-2     //25ms

// buffer used to store the msgs
// std::vector<sensor_msgs::CameraInfo> mono_info_buff(0);
std::vector<sensor_msgs::Image> mono_image_buff(0);
std::vector<quadrotor_msgs::GPIOTime> time_buff(0);
// std::vector<sensor_msgs::Imu> imu_buff(0);

// std::vector<sensor_msgs::CameraInfo>::iterator mono_info_iter;
std::vector<sensor_msgs::Image>::iterator mono_image_iter;
std::vector<quadrotor_msgs::GPIOTime>::iterator time_iter;
// std::vector<sensor_msgs::Imu>::iterator imu_iter;

std::vector<double> compare_vector(2);
std::vector<double>::iterator compare_iter;
std::vector<double>::iterator max_iter;
std::vector<double>::iterator min_iter;

// unsigned int mono_counter = 0;
// unsigned int imu_counter = 0;
// unsigned int time_counter = 0;

// unsigned int mono_counter_one = 0;
// unsigned int imu_counter_one = 0;

double time_error = 0;


void gpio_time_callback(const quadrotor_msgs::GPIOTime::ConstPtr& time)
{
    time_error = fabs(time->ref1.stamp.toSec() - time->ref2.stamp.toSec());
    
    if (time_error > 1e-3)
    {   
        // ROS_WARN("gpio_time time_error is too big.");
        return;
    }
    // valid gpio_time message
    else
    {
        time_buff.push_back(*time);
        ROS_WARN("1 gpio_time is added.");
    }
}

void mono_image_callback(const sensor_msgs::ImageConstPtr& image)
{
    mono_image_buff.push_back(*image);
    ROS_WARN("1 mono_image is added.");

}

// void imu_callback(const sensor_msgs::ImuConstPtr& imu)
// {
//     // pub_imu.publish(*imu);

// }

//=======================================================================================
// correct the mono image time stamp with px4-gpio time domain
//=======================================================================================

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sync_mono_imu_post");
    ros::NodeHandle nh;

    std::string ns = nh.getNamespace();

    // std::string sub_imu_topic_name = "/mavlink/imu";
    std::string sub_time_topic_name = "/mavlink/gpio_time";
    std::string sub_mono_image_topic_name = "/mono/image_rect";

    // std::string pub_imu_topic_name = ros::names::append(ns, "sync/imu");
    std::string pub_time_topic_name = ros::names::append(ns, "sync/gpio_time");
    std::string pub_mono_image_topic_name = ros::names::append(ns, "sync/mono/image_rect");

    image_transport::ImageTransport it(nh);
    
    // ros::Subscriber sub_imu = 
    //     nh.subscribe(sub_imu_topic_name, 1, imu_callback); 
    ros::Subscriber sub_time = 
        nh.subscribe(sub_time_topic_name, 1, gpio_time_callback);
    image_transport::Subscriber sub_mono_image = 
        it.subscribe(sub_mono_image_topic_name, 1, mono_image_callback);
    
    //image_transport::Publisher pub_left_image = it.advertise(pub_left_image_topic_name, 1);
    //image_transport::Publisher pub_right_image = it.advertise(pub_right_image_topic_name, 1);
    ros::Publisher pub_time = 
            nh. advertise<quadrotor_msgs::GPIOTime>(pub_time_topic_name, 5);
    ros::Publisher pub_mono_image = 
            nh.advertise<sensor_msgs::Image>(pub_mono_image_topic_name, 5);
    // ros::Publisher pub_imu = 
    //         nh.advertise<sensor_msgs::Imu>(pub_imu_topic_name, 5);

    ros::Rate rate(100);

    while(ros::ok())
    {
        
        while(  (mono_image_buff.size() > 0) &&  
                (time_buff.size() > 0) )
        {
            compare_vector.at(0) = time_buff[0].header.stamp.toSec();
            compare_vector.at(1) = mono_image_buff[0].header.stamp.toSec();

            compare_iter = compare_vector.begin();

            max_iter = max_element(compare_iter, compare_iter+2);
            min_iter = min_element(compare_iter, compare_iter+2);

            // three time stamps match
            if(*max_iter - *min_iter < IMAGE_FRAME_TIME_TOLERANCE)
            {
                //change time of image
                mono_image_buff.at(0).header.stamp = time_buff.at(0).ref1.stamp;

                // publish the msgs
                pub_time.publish(time_buff.at(0));
                pub_mono_image.publish(mono_image_buff.at(0));

                // delete the published msgs from the vector
                time_buff.erase(time_buff.begin());
                mono_image_buff.erase(mono_image_buff.begin());

                ROS_WARN("Output a sync.");
            }
            // there are at least one stamp-time that is too old which may be caused by frame/message lost
            // then delete the oldest message
            else
            {
                if(min_iter == compare_iter)
                {
                    time_buff.erase(time_buff.begin());
                    ROS_WARN("GPIO-Time message is too old!");
                }
                else if(min_iter == compare_iter+1)
                {
                    mono_image_buff.erase(mono_image_buff.begin());
                    ROS_WARN("Mono-image message is too old!");
                }

            }

        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


