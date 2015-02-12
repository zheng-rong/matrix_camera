// simple node which interacts with the camera settings

#include "ros/ros.h"
#include <cstdlib>
#include <string>
//#include "cv.h"
#include <mv_camera/PropertyMap.h>

//#include <cv_bridge/cv_bridge.h>
//#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>
//#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>

using namespace std;
//usign namespace cv;
namespace enc = sensor_msgs::image_encodings;

typedef mv_camera::PropertyMap::Request Request;

bool saveImageToFileBool = 1;
bool continuousCapture = 1;

int sendAndPrintResult(ros::ServiceClient client, mv_camera::PropertyMap srv)
{

  if (client.call(srv))
  {
    if (srv.response.status == 0)
    {
      cout << "An ERROR occured:" << endl;
    }
    cout << srv.response.result << endl;
    return srv.response.status;

  }
  else
  {
    ROS_ERROR("Failed to call service!");
    cout << "Failed to call service!";
    return 0;
  }

}

string getInputForQuestion(string question)
{
  cout << question << endl;
  char buffer[4096];
  string out;
  cin.getline(buffer,4096);
  out = buffer;

  return out;
}

//void saveImageToFile(const sensor_msgs::ImageConstPtr& msg, string filename) {
//
//    // convert the mesasge to cv image pointer
//    cv_bridge::CvImagePtr cv_ptr;
//    try
//    {
//        cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
//    }
//    catch (cv_bridge::Exception& e)
//    {
//        ROS_ERROR("cv_bridge exception: %s", e.what());
//        return;
//    }
//    cv::imwrite(filename, cv_ptr->image);
//
//}
//
//
//
//void imageMessageCallback(const sensor_msgs::ImageConstPtr& msg)
//{
//    if (saveImageToFileBool && !continuousCapture) {
//
//        std::time_t t = std::time(0);
//        std::stringstream out;
//        out << t;
//        saveImageToFile(msg, "images/im_" + out.str() + ".jpg" );
//        cout << "Received and saved Image to: " << "images/im_" + out.str() + ".jpg";
//
//    }
//}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "driver_interface");

  cout << "Initialising ROS and service client side...\n";

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<mv_camera::PropertyMap>("poll_property_list");
  // create a listener
//    ros::Subscriber sub = n.subscribe("/camera/image_raw", 1000, imageMessageCallback);
  // asynchronous spinning to save the images in the background
  ros::AsyncSpinner spinner(4);
  spinner.start();

  cout << "Done.\n";
  cout << "Type \"help\" to get a list of commands\n";
  string cmd;

  // the main loop:
  bool boRun = true;
  while (boRun && ros::ok())
  {

    mv_camera::PropertyMap srv;
    // clean request:
    srv.request.command = 0;
    srv.request.identifier = "";
    srv.request.value = "";

    std::cout << "> ";
    // read command:
    char buffer[4096];
    cin.getline(buffer,4096);
    //cin >> cmd;
    cmd = buffer;
    /// QUIT
    if (cmd == "quit" || cmd == "q")
    {
      boRun = false;
      continue;
    }

    // check if service still exists:
    if (!client.exists())
    {
      cout << "Service currently unavailable!" << endl;
      cout << "Trying to connect to: " << n.resolveName("poll_property_list", true) << std::endl;
      client = n.serviceClient<mv_camera::PropertyMap>(n.resolveName("poll_property_list", true));
      continue;
    }

    // chose command.
    /// LIST
    if (cmd == "list" || cmd == "l")
    {
      // request setup:
      srv.request.command = Request::GET_PROPERTY_LIST;
      sendAndPrintResult(client, srv);
    }
    /// FIND
    else if (cmd == "find" || cmd == "f")
    {
      srv.request.command = Request::SEARCH_PROPERTY_MAP;
      srv.request.identifier = getInputForQuestion("Enter Search String:");

      sendAndPrintResult(client, srv);

    }
    /// SET
    else if (cmd == "set" || cmd == "s")
    {
      srv.request.identifier = getInputForQuestion("Enter Property Identifier:");

      // send a request to the server to get the info:
      srv.request.command = Request::GET_PROPERTY_INFO;
      int status = sendAndPrintResult(client, srv);

      if (status == 0)
      {
        cout << "This property does not exist!";
      }
      else
      {
        // now continue and ask for the value:
        srv.request.command = Request::SET_PROPERTY;
        srv.request.value = getInputForQuestion("Enter new Property Value:");
        // send parameter set request:
        sendAndPrintResult(client, srv);
      }
    }
    /// INFO
    else if (cmd == "info" || cmd == "i")
    {
      srv.request.command = Request::GET_PROPERTY_INFO;
      srv.request.identifier = getInputForQuestion("Enter Property Identifier:");
      sendAndPrintResult(client, srv);
    }
    /// SAVE
    else if (cmd == "save")
    {
      srv.request.command = Request::SAVE_SETTINGS;
      srv.request.identifier = getInputForQuestion("File Path:");
      sendAndPrintResult(client, srv);
    }
    /// LOAD
    else if (cmd == "load")
    {
      srv.request.command = Request::LOAD_SETTINGS;
      srv.request.identifier = getInputForQuestion("File Path:");
      sendAndPrintResult(client, srv);
    }
    else if (cmd == "capture" || cmd == "c")
    {
      srv.request.command = Request::START_CAPTURE_PROCESS;
      sendAndPrintResult(client, srv);
      continuousCapture = 1;
    }
    else if (cmd == "stop" || cmd == "s")
    {
      srv.request.command = Request::STOP_CAPTURE_PROCESS;
      sendAndPrintResult(client, srv);
      continuousCapture = 0;
    }
    else if (cmd == "single")
    {
      srv.request.command = Request::CAPTURE_SINGLE_FRAME;
      sendAndPrintResult(client, srv);
    }
    else if (cmd == "restart")
    {
      srv.request.command = Request::RESTART_DEVICE;
      sendAndPrintResult(client, srv);
    }
    else if (cmd == "close")
    {
      srv.request.command = Request::CLOSE_DEVICE;
      sendAndPrintResult(client, srv);
    }
    else if (cmd == "open")
    {
      srv.request.command = Request::OPEN_DEVICE;
      sendAndPrintResult(client, srv);
    }
    /// HELP
    else
    {
      cout << "Commands are: \n";
      cout << "\t quit: exit the program \n";
      cout << "\t list: list all available properties \n";
      cout << "\t set: set a property \n";
      cout << "\t info: get property details \n";
      cout << "\t save: save camera configuration \n";
      cout << "\t load: load camera configuration \n";
      cout << "\t capture: start continuous capture\n";
      cout << "\t stop: stop continuous capture \n";
      cout << "\t single: get a single image \n";
      cout << "\t restart: close and reopen device \n";
      cout << "\t close: close device \n";
      cout << "\t open: open device \n";
      cout << "\t find: find a parameter by substring \n";
    }

    cout << "\n------\n";
  }
  return 0;
}
