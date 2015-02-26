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
        ROS_INFO(">> %s",srv.request.identifier.c_str());
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

bool listProperty(ros::ServiceClient& client)
{
    mv_camera::PropertyMap srv;

    // Fill in the request
    srv.request.command = mv_camera::PropertyMap::Request::GET_PROPERTY_LIST;


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
    ros::init(argc, argv, "modify_mono_settings");
    ros::NodeHandle n;

    std::string ns = n.getNamespace();
    std::string mono_cam_service  = ros::names::append(ns, "mono/poll_property_list");

    ros::service::waitForService(mono_cam_service);

    ros::ServiceClient client_mono = n.serviceClient<mv_camera::PropertyMap>(mono_cam_service);

    string identifier;
    string value;
    
    bool param;
    std::string key;


    ROS_WARN("Trigger Mode Setting:");

    if(!ros::param::search("ext_trig",key)) 
    {
        ROS_ERROR("Failed to search for the parameter 'ext_trig'");
        return false;
    }

    if(!ros::param::has(key))
    {
        ROS_ERROR("Can not find the parameter %s", key.c_str());
        return false;
    }

    if(!ros::param::get(key, param))
    {
        ROS_ERROR("Failed to get the parameter %s", key.c_str());
        return false;
    }
        
    std::cout << key << ":\t" << param << std::endl;


    /**********************************************************************
     *                           Modify Settings
     * *******************************************************************/

//    listProperty(client_mono);


    identifier = "Camera/PixelClock_KHz";
    value = "40000";
    queryService(client_mono, identifier, value);
    callService(client_mono, identifier, value);

    identifier = "Camera/ShutterMode";
    value = "FrameShutter";
    queryService(client_mono, identifier, value);
    callService(client_mono, identifier, value);

    identifier = "Camera/Expose_us";
    value = "10000";
    queryService(client_mono, identifier, value);
    
    identifier = "Camera/ExposeMode";
    value = "Standard"; //"Overlapped";     //Overlapped
    queryService(client_mono, identifier, value);
    callService(client_mono, identifier, value);
/*
    identifier = "Camera/Framerate_Hz";
    value = "50";
    queryService(client_mono, identifier, value);
    callService(client_mono, identifier, value);
*/
    // Set the Binning mode to reduce the image to one quarter
    identifier = "Camera/BinningMode";
    value = "BinningHV";    //"Off";  //"BinningHV";
    queryService(client_mono, identifier, value);
    callService(client_mono, identifier, value);

    // Enable Autoexposure
    identifier = "Camera/AutoExposeControl";
    value = "On";
    queryService(client_mono,identifier, value);
    callService(client_mono, identifier, value);

    // Enable Autogain
    identifier = "Camera/AutoGainControl";
    value = "On";
    callService(client_mono, identifier, value);

    // Set the region used to adjust the exposure and gain
    identifier = "Camera/AutocontrolParameters/AoiMode";
    value = "Full";
    callService(client_mono, identifier, value);

    // Set the auto-control mode
    identifier = "Camera/AutoControlMode";
    value = "DeviceSpecific";
    callService(client_mono, identifier, value);

    // Set the auto-control speed
    identifier = "Camera/AutoControlParameters/ControllerSpeed";
    value = "Fast";
    callService(client_mono, identifier, value);

    // Disable the HDR mode
    identifier = "Camera/HDRControl/HDREnable";
    value = "Off";
    callService(client_mono, identifier, value);
    
    if(param)
    {
        // Set the mono camera to slave mode
        identifier = "Camera/TriggerMode";
        value = "OnHighLevel";
        queryService(client_mono, identifier, value);
        callService(client_mono, identifier, value);

        identifier = "Camera/TriggerSource";
        value = "DigIn0";
        callService(client_mono, identifier, value);  
    }
    else
    {
         // Set the left camera to master mode
        identifier = "Camera/FlashMode";
        value = "Digout0";
        callService(client_mono, identifier, value);

        identifier = "Camera/FlashType";
        value = "Standard";
        callService(client_mono, identifier, value);
    }

    return 0;

}



