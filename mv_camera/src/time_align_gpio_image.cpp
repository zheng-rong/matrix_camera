#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <quadrotor_msgs/GPIOTime.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>


using namespace std;

#define FRAME_TIME_TOLERANCE 3.0e-2     //25ms

/*
bool left_image_ready = false;
bool right_image_ready = false;
bool gpio_time_ready = false;

// temp buff used to store the msg
sensor_msgs::Image left_image_msg;
sensor_msgs::Image right_image_msg;
quadrotor_msgs::GPIOTime gpio_time_msg;

// previous corresponding msgs: left, right, gpio
sensor_msgs::Image          prev_left_image;
sensor_msgs::Image          prev_right_image;
quadrotor_msgs::GPIOTime    prev_gpio_time;

//current corresponding msgs: left, right, gpio
sensor_msgs::Image          curr_left_image;
sensor_msgs::Image          Curr_right_image;
quadrotor_msgs::GPIOTime    curr_gpio_time;

ros::Time prev_approx_time;
ros::Time curr_approx_time;

unsigned int prev_msgs_num = 0;

bool prev_left_ready = false;
bool prev_right_ready = false;
bool prev_gpio_ready = false;

unsigned int updated_image_num = 0;

*/

bool align_msgs_ready = false;

// buffer used to store the msgs
std::vector<sensor_msgs::CameraInfo> left_info_buff(0);
std::vector<sensor_msgs::CameraInfo> right_info_buff(0);
std::vector<sensor_msgs::Image> left_image_buff(0);
std::vector<sensor_msgs::Image> right_image_buff(0);
std::vector<quadrotor_msgs::GPIOTime> time_buff(0);

std::vector<sensor_msgs::CameraInfo>::iterator left_info_iter;
std::vector<sensor_msgs::CameraInfo>::iterator right_info_iter;
std::vector<sensor_msgs::Image>::iterator left_image_iter;
std::vector<sensor_msgs::Image>::iterator right_image_iter;
std::vector<quadrotor_msgs::GPIOTime>::iterator time_iter;

std::vector<double> compare_vector(5);
std::vector<double>::iterator compare_iter;
std::vector<double>::iterator max_iter;
std::vector<double>::iterator min_iter;

double time_error = 0;

void gpio_time_callback(const quadrotor_msgs::GPIOTime::ConstPtr& time)
{
    // find out the valid gpio-time message with corresponding ref1&ref2 time
    // the time error should be close to 0ms or 50ms in the case of 20Hz frame rate 
    // the threshould is set to 1ms
    // useful  message: error < 1ms
    // useless message: 49ms < error < 51ms
    // odd message: 1ms < error < 49ms or error > 51ms, should NOT happen! 
    // invalid gpio_time message
    time_error = fabs(time->ref1.stamp.toSec() - time->ref2.stamp.toSec());
    if (time_error > 0.1e-2)
    {
	    if((time_error < 4.9e-2)||(time_error > 5.3e-2))
        {
            ROS_INFO("Odd GPIO-time message: left - right = %f", time_error);
        }
        
        return;
    }
    // valid gpio_time message
    else
    {
        time_buff.push_back(*time);

    }
}

void left_image_callback(const sensor_msgs::ImageConstPtr& image)
{
    cv_bridge::CvImagePtr img_temp = cv_bridge::toCvCopy(image);
    cv::flip(img_temp->image, img_temp->image, -1);
    left_image_buff.push_back(*(img_temp->toImageMsg()));
}

void right_image_callback(const sensor_msgs::ImageConstPtr& image)
{
    cv_bridge::CvImagePtr img_temp = cv_bridge::toCvCopy(image);
    cv::flip(img_temp->image, img_temp->image, -1);
    right_image_buff.push_back(*(img_temp->toImageMsg()));
}

void left_info_callback(const sensor_msgs::CameraInfoConstPtr& info)
{
    left_info_buff.push_back(*info);
}

void right_info_callback(const sensor_msgs::CameraInfoConstPtr& info)
{
    right_info_buff.push_back(*info);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "time_align_gpio_image");
    ros::NodeHandle nh;

    std::string ns = nh.getNamespace();

    std::string sub_time_topic_name =  "/mavlink/gpio_time";
    std::string sub_left_image_topic_name = ros::names::append(ns, "stereo/left/image_raw");
    std::string sub_right_image_topic_name = ros::names::append(ns, "stereo/right/image_raw");
    std::string sub_left_info_topic_name = ros::names::append(ns, "stereo/left/camera_info");
    std::string sub_right_info_topic_name = ros::names::append(ns, "stereo/right/camera_info");

    std::string pub_time_topic_name = ros::names::append(ns, "align/gpio_time");
    std::string pub_left_image_topic_name = ros::names::append(ns, "align/stereo/left/image_raw");
    std::string pub_right_image_topic_name = ros::names::append(ns, "align/stereo/right/image_raw");
    std::string pub_left_info_topic_name = ros::names::append(ns, "align/stereo/left/camera_info");
    std::string pub_right_info_topic_name = ros::names::append(ns, "align/stereo/right/camera_info");

    image_transport::ImageTransport it(nh);
    
    
    image_transport::Subscriber sub_left_image = it.subscribe(sub_left_image_topic_name, 1, left_image_callback);
    image_transport::Subscriber sub_right_image = it.subscribe(sub_right_image_topic_name, 1, right_image_callback);
    
    ros::Subscriber sub_time = nh.subscribe(sub_time_topic_name, 1, gpio_time_callback);
    ros::Subscriber sub_left_info = nh.subscribe(sub_left_info_topic_name, 1, left_info_callback);
    ros::Subscriber sub_right_info = nh.subscribe(sub_right_info_topic_name, 1, right_info_callback);



    ros::Publisher pub_time = nh. advertise<quadrotor_msgs::GPIOTime>(pub_time_topic_name, 5);
    
    //image_transport::Publisher pub_left_image = it.advertise(pub_left_image_topic_name, 1);
    //image_transport::Publisher pub_right_image = it.advertise(pub_right_image_topic_name, 1);
    ros::Publisher pub_left_image = nh.advertise<sensor_msgs::Image>(pub_left_image_topic_name, 5);
    ros::Publisher pub_right_image = nh.advertise<sensor_msgs::Image>(pub_right_image_topic_name, 5);
    ros::Publisher pub_left_info = nh.advertise<sensor_msgs::CameraInfo>(pub_left_info_topic_name, 5);
    ros::Publisher pub_right_info = nh.advertise<sensor_msgs::CameraInfo>(pub_right_info_topic_name, 5);

    ros::Rate rate(100); //40Hz

    while(ros::ok())
    {

        while(  (left_image_buff.size() > 0) && 
                (right_image_buff.size() > 0) && 
                (time_buff.size() > 0) && 
                (left_info_buff.size() > 0) && 
                (right_info_buff.size() > 0))
        {
            compare_vector.at(0) = time_buff[0].header.stamp.toSec();
            compare_vector.at(1) = left_image_buff[0].header.stamp.toSec();
            compare_vector.at(2) = right_image_buff[0].header.stamp.toSec();
            compare_vector.at(3) = left_info_buff[0].header.stamp.toSec();
            compare_vector.at(4) = right_info_buff[0].header.stamp.toSec();

            compare_iter = compare_vector.begin();

            max_iter = max_element(compare_iter, compare_iter+5);
            min_iter = min_element(compare_iter, compare_iter+5);

            // three time stamps match
            if(*max_iter - *min_iter < FRAME_TIME_TOLERANCE)
            {
                //change time of image and info
                left_image_buff.at(0).header.stamp = time_buff.at(0).ref1.stamp;
                right_image_buff.at(0).header.stamp = time_buff.at(0).ref2.stamp;
                left_info_buff.at(0).header.stamp = time_buff.at(0).ref1.stamp;
                right_info_buff.at(0).header.stamp = time_buff.at(0).ref2.stamp;

                // publish the msgs
                pub_time.publish(time_buff.at(0));
                pub_left_image.publish(left_image_buff.at(0));
                pub_right_image.publish(right_image_buff.at(0));
                pub_left_info.publish(left_info_buff.at(0));
                pub_right_info.publish(right_info_buff.at(0));

                // erase the published msgs from the vector
                time_buff.erase(time_buff.begin());
                left_image_buff.erase(left_image_buff.begin());
                right_image_buff.erase(right_image_buff.begin());
                left_info_buff.erase(left_info_buff.begin());
                right_info_buff.erase(right_info_buff.begin());
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
                    left_image_buff.erase(left_image_buff.begin());
                    ROS_WARN("Left-image message is too old!");
                }
                else if(min_iter == compare_iter+2)
                {
                    right_image_buff.erase(right_image_buff.begin());
                    ROS_WARN("Right-image message is too old!");
                }
                else if(min_iter == compare_iter+3)
                {
                    left_info_buff.erase(left_info_buff.begin());
                    ROS_WARN("Left-info message is too old!");
                }
                else if(min_iter == compare_iter+4)
                {
                    right_info_buff.erase(right_info_buff.begin());
                    ROS_WARN("Right-info message is too old!");
                }

                //ROS_WARN("Messages of Stereo-Image or GPIO-Time may be lost!");
            }

        }


        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}






















