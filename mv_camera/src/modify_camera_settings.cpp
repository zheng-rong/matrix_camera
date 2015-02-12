#include <iostream>
#include <ros/ros.h>
#include <string>
#include <mvIMPACT_CPP/mvIMPACT_acquire.h>
#include <mv_camera/PropertyMap.h>


using namespace std;


bool callService(ros::ServiceClient& client, string& identifier, string& value)
{
    mv_camera::PropertyMap srv;

    // Fill in the request
    srv.request.command = mv_camera::PropertyMap::Request::SET_PROPERTY;
    srv.request.identifier = identifier;
    srv.request.value = value;

    // Call the service
    if (client.call(srv))
    {
        ROS_INFO("Status: %d", srv.response.status);
        ROS_INFO("Result: %s", srv.response.result.c_str());
        return true;
    }
    else
    {
        ROS_ERROR("Failed to call the service \"poll_property_list\"!");
        return false;
    }
}

bool queryService(ros::ServiceClient& client, string& identifier, string& value)
{
    mv_camera::PropertyMap srv;

    // Fill in the request
    srv.request.command = mv_camera::PropertyMap::Request::GET_PROPERTY_INFO;
    srv.request.identifier = identifier;
    srv.request.value = value;

    // Call the service
    if (client.call(srv))
    {
        ROS_INFO("Status: %d", srv.response.status);
        ROS_INFO("Result: %s", srv.response.result.c_str());
        return true;
    }
    else
    {
        ROS_ERROR("Failed to call the service \"poll_property_list\"!");
        return false;
    }
}

int main(int argc, char **argv)
{

    // Initialize the client
    ros::init(argc, argv, "modify_property_list");
    ros::NodeHandle n;

    std::string ns = n.getNamespace();
    std::string left_cam_service  = ros::names::append(ns, "stereo/left/poll_property_list");
    std::string right_cam_service = ros::names::append(ns, "stereo/right/poll_property_list");

    ros::service::waitForService(left_cam_service);
    ros::service::waitForService(right_cam_service);

    ros::ServiceClient client_left = n.serviceClient<mv_camera::PropertyMap>(left_cam_service);
    ros::ServiceClient client_right = n.serviceClient<mv_camera::PropertyMap>(right_cam_service);

    string identifier;
    string value;

    /**********************************************************************
     *                           Modify Settings
     * *******************************************************************/
    // Set the Binning mode to reduce the image to one quarter
    identifier = "Camera/BinningMode";
    value = "BinningHV";
    callService(client_left, identifier, value);
    callService(client_right, identifier, value);

    // Enable Autoexposure
    identifier = "Camera/AutoExposeControl";
    value = "On";
    callService(client_left, identifier, value);
    callService(client_right, identifier, value);

    // Enable Autogain
    identifier = "Camera/AutoGainControl";
    value = "On";
    callService(client_left, identifier, value);
    callService(client_right, identifier, value);

    // Set the region used to adjust the exposure and gain
    identifier = "Camera/AutocontrolParameters/AoiMode";
    value = "Full";
    callService(client_left, identifier, value);
    callService(client_right, identifier, value);

    // Set the auto-control mode
    identifier = "Camera/AutoControlMode";
    value = "DeviceSpecific";
    callService(client_left, identifier, value);
    callService(client_right, identifier, value);

    // Set the auto-control speed
    identifier = "Camera/AutoControlParameters/ControllerSpeed";
    value = "Fast";
    callService(client_left, identifier, value);
    callService(client_right, identifier, value);

    // Disable the HDR mode
    identifier = "Camera/HDRControl/HDREnable";
    value = "Off";
    callService(client_left, identifier, value);
    callService(client_right, identifier, value);


/*
    // Set the left camera to master mode
    identifier = "Camera/FlashMode";
    value = "Digout0";
    callService(client_left, identifier, value);

    identifier = "Camera/FlashType";
    value = "Standard";
    callService(client_left, identifier, value);
*/

    // Set the right camera to slave mode
    identifier = "Camera/TriggerMode";
    value = "OnHighLevel";
    queryService(client_left, identifier, value);
    callService(client_left, identifier, value);

    queryService(client_right, identifier, value);
    callService(client_right, identifier, value);

    identifier = "Camera/TriggerSource";
    value = "DigIn0";
    callService(client_right, identifier, value);
    
    callService(client_left, identifier, value);


    return 0;

}



