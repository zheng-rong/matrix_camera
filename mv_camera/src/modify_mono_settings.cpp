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
    std::string mono_cam_service  = ros::names::append(ns, "poll_property_list");
    // ros::names::append(ns, "poll_property_list");

    ROS_WARN("====== waitForService...");
    ROS_WARN(mono_cam_service.c_str());

    ros::service::waitForService(mono_cam_service);

    ROS_WARN("====== waitForService done.");

    ros::ServiceClient client_mono = n.serviceClient<mv_camera::PropertyMap>(mono_cam_service);

    string identifier;
    string value;
    string key;
    
    bool param_trigger;
    std::string param_auto_expo;
    std::string param_hdr_en;
    std::string param_binn_mode;
    std::string param_auto_gain;
    std::string param_expo_mode;
    int param_expo_time;
    

    ROS_WARN("Try to read configuration parameters...");
    //===============================================================
    ROS_WARN("Trigger Mode Setting:");
    if(!ros::param::search("ext_trig",key)) 
        {ROS_ERROR("Failed to search for the parameter 'ext_trig'"); return false;}
    if(!ros::param::has(key))
        {ROS_ERROR("Can not find the parameter %s", key.c_str());    return false;}
    if(!ros::param::get(key, param_trigger))
        {ROS_ERROR("Failed to get the parameter %s", key.c_str());   return false;}    
    std::cout << key << ":\t" << param_trigger << std::endl;

    //===============================================================
    ROS_WARN("Setting Binning Mode:");
    if(!ros::param::search("binn_mode",key)) 
        {ROS_ERROR("Failed to search for the parameter 'binn_mode'");  return false;}
    if(!ros::param::has(key))
        {ROS_ERROR("Can not find the parameter %s", key.c_str());    return false;}
    if(!ros::param::get(key, param_binn_mode))
        {ROS_ERROR("Failed to get the parameter %s", key.c_str());   return false;}  
    std::cout << key << ":\t" << param_binn_mode << std::endl;

    //===============================================================
    ROS_WARN("Setting Auto Expose:");
    if(!ros::param::search("auto_expo",key)) 
        {ROS_ERROR("Failed to search for the parameter 'auto_expo'");  return false;}
    if(!ros::param::has(key))
        {ROS_ERROR("Can not find the parameter %s", key.c_str());    return false;}
    if(!ros::param::get(key, param_auto_expo))
        {ROS_ERROR("Failed to get the parameter %s", key.c_str());   return false;} 
    std::cout << key << ":\t" << param_auto_expo << std::endl;

    //===============================================================
    ROS_WARN("Setting Expose Mode:");
    if(!ros::param::search("expo_mode",key)) 
        {ROS_ERROR("Failed to search for the parameter 'expo_mode'");  return false;}
    if(!ros::param::has(key))
        {ROS_ERROR("Can not find the parameter %s", key.c_str());    return false;}
    if(!ros::param::get(key, param_expo_mode))
        {ROS_ERROR("Failed to get the parameter %s", key.c_str());   return false;}  
    std::cout << key << ":\t" << param_expo_mode << std::endl;

    //===============================================================
    ROS_WARN("Setting Expose Time:");
    if(!ros::param::search("expo_time",key)) 
        {ROS_ERROR("Failed to search for the parameter 'expo_time'");  return false;}
    if(!ros::param::has(key))
        {ROS_ERROR("Can not find the parameter %s", key.c_str());    return false;}
    if(!ros::param::get(key, param_expo_time))
        {ROS_ERROR("Failed to get the parameter %s", key.c_str());   return false; }  
    std::cout << key << ":\t" << param_expo_time << std::endl;

    //===============================================================
    ROS_WARN("Setting Gain Mode:");
    if(!ros::param::search("auto_gain",key)) 
        {ROS_ERROR("Failed to search for the parameter 'auto_gain'");  return false;}
    if(!ros::param::has(key))
        {ROS_ERROR("Can not find the parameter %s", key.c_str());    return false;}
    if(!ros::param::get(key, param_auto_gain))
        {ROS_ERROR("Failed to get the parameter %s", key.c_str());   return false; } 
    std::cout << key << ":\t" << param_auto_gain << std::endl;

    //===============================================================
    ROS_WARN("Setting HDR Mode:");
    if(!ros::param::search("hdr_mode",key)) 
        {ROS_ERROR("Failed to search for the parameter 'hdr_mode'");  return false;}
    if(!ros::param::has(key))
        {ROS_ERROR("Can not find the parameter %s", key.c_str());    return false;}
    if(!ros::param::get(key, param_hdr_en))
        {ROS_ERROR("Failed to get the parameter %s", key.c_str());   return false;}   
    std::cout << key << ":\t" << param_hdr_en << std::endl;

    /**********************************************************************
     *                           Modify Settings
     * *******************************************************************/

    ROS_WARN("Try to set the hard-coded parameters...");

//    listProperty(client_mono);


    identifier = "Camera/PixelClock_KHz";
    value = "40000";
    queryService(client_mono, identifier, value);
    callService(client_mono, identifier, value);

    identifier = "Camera/ShutterMode";
    value = "FrameShutter";
    queryService(client_mono, identifier, value);
    callService(client_mono, identifier, value);
    
/*
    identifier = "Camera/Framerate_Hz";
    value = "50";
    queryService(client_mono, identifier, value);
    callService(client_mono, identifier, value);
*/
    // Set the region used to adjust the exposure and gain
    identifier = "Camera/AutocontrolParameters/AoiMode";
    value = "Full";
    queryService(client_mono, identifier, value);
    callService(client_mono, identifier, value);

    // Set the auto-control mode
    identifier = "Camera/AutoControlMode";
    value = "Standard"; //"DeviceSpecific";
    queryService(client_mono, identifier, value);
    callService(client_mono, identifier, value);

    // Set the auto-control speed
    identifier = "Camera/AutoControlParameters/ControllerSpeed";
    value = "Fast"; //"Low";  //"Medium";   //"Fast";
    queryService(client_mono, identifier, value);
    callService(client_mono, identifier, value);

    //============================================================
    //============================================================
    //============================================================
    ROS_WARN("Try to set the configured parameters...");

    // Set the Binning mode to reduce the image to one quarter
    identifier = "Camera/BinningMode";
    value = param_binn_mode;    //"Off";  //"BinningHV";
    queryService(client_mono, identifier, value);
    callService(client_mono, identifier, value);

    // set the expose mode
    identifier = "Camera/ExposeMode";
    value = param_expo_mode;                    //"Standard"; //"Overlapped"
    queryService(client_mono, identifier, value);
    callService(client_mono, identifier, value);

    // set the Autoexposure
    identifier = "Camera/AutoExposeControl";
    value = param_auto_expo;
    queryService(client_mono,identifier, value);
    callService(client_mono, identifier, value);

    // set the Autogain
    identifier = "Camera/AutoGainControl";
    value = param_auto_gain;
    queryService(client_mono,identifier, value);
    callService(client_mono, identifier, value);

    // set up the HDR mode
    identifier = "Camera/HDRControl/HDREnable";
    value = param_hdr_en;
    queryService(client_mono,identifier, value);
    callService(client_mono, identifier, value);
 
    if(param_trigger)
    {
        // Set the mono camera to slave mode
        identifier = "Camera/TriggerMode";
        value = "OnHighLevel";
        queryService(client_mono, identifier, value);
        callService(client_mono, identifier, value);

        identifier = "Camera/TriggerSource";
        value = "DigIn0";
        queryService(client_mono,identifier, value);
        callService(client_mono, identifier, value);  
    }
    else
    {
         // Set the left camera to master mode
        identifier = "Camera/FlashMode";
        value = "Digout0";
        queryService(client_mono,identifier, value);
        callService(client_mono, identifier, value);

        identifier = "Camera/FlashType";
        value = "Standard";
        queryService(client_mono,identifier, value);
        callService(client_mono, identifier, value);
    }

    // set the exposion time
    identifier = "Camera/Expose_us";
    value = param_expo_time;                        //"10000";
    queryService(client_mono, identifier, value);

    return 0;
}



