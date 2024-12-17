// #include "ros/ros.h"
// #include "std_msgs/String.h"
// #include "std_msgs/Bool.h"
// #include <string>
// #include <map>
// #include <iostream>
// #include "flirBFS.h"
// #include <json/json.h>
// //#include <jsoncpp/json/json.h>
// #include <json/value.h>

// #include <chrono>  // chrono::system_clock
// #include <ctime>   // localtime
// #include <sstream> // stringstream
// #include <iomanip> // put_time


// //ros publish image
// #include<image_transport/image_transport.h>
// #include<cv_bridge/cv_bridge.h>

// using namespace std;

// class Param_listener: public BFSMulti
// {

// public:
//     Param_listener(rclcpp::NodeHandle* nodehandle); 

//     ~Param_listener();
    
//     std::map<string, string> params_rev;

// 	SystemPtr system;

// 	CameraList camList;
	
// 	std::map<string, std::map<string, string>> paras_cams;

// 	boost::shared_ptr<BFSMulti> pCameras{new BFSMulti()}; 
	
// 	std::thread _threadWorker;

//     rclcpp::Time ros_time; 
// 	double rostimeSec;
	
// 	string str;
// private:

//     rclcpp::NodeHandle nh_;
//     rclcpp::NodeHandle nhi_;

//     auto nh_ = rclcpp::Node::make_shared("nh_");
//     auto nhi_ = rclcpp::Node::make_shared("nhi_");

// 	rclcpp::Subscriber connect_sub;
//     rclcpp::Subscriber cont_trigger_sub;
// 	rclcpp::Subscriber stop_cont_trigger_sub;
//     rclcpp::Subscriber disconnect_sub;
//     rclcpp::Subscriber frame_rate_sub;
// 	rclcpp::Subscriber capture_sub;
// 	rclcpp::Subscriber exposureTime_sub;
//     rclcpp::Subscriber start_preview_sub;
//     rclcpp::Subscriber stop_preview_sub;

//     void createDayFolder();

//     string CurrentDayFolder;

//     string getCurrentDateStr();

// 	void initParamMap(); 
// 	void listen_param();

// 	void connectCallback(const std_msgs::msg::String::SharedPtr msg);
// 	void stopContTriggerCallback(const std_msgs::msg::String::SharedPtr msg);
// 	void contTriggerCallback(const std_msgs::msg::String::SharedPtr msg);
// 	void disconnectCallback(const std_msgs::msg::String::SharedPtr msg);
// 	void frameRateCallback(const std_msgs::msg::String::SharedPtr msg);
// 	void captureCallback(const std_msgs::msg::String::SharedPtr msg);
// 	void exposureTimeCallback(const std_msgs::msg::String::SharedPtr msg);
// 	void startPreviewCallback(const std_msgs::msg::String::SharedPtr msg);
//     void stopPreviewCallback(const std_msgs::msg::String::SharedPtr msg);
// 	void preview();

// 	string dataPath = "/media/nvidia/Data2/"; ///home/nvidia/catkin_ws/Data/

// 	bool systemCreated = false;
	
// 	image_transport::Publisher pub, pub1, pub2, pub3, pub4, pub5, pub6;
//     std::vector<image_transport::Publisher> pubvector;
    
//     int cam_count;
    
//     double frame_rate=10.f;

// 	int sleep_time=100;
// };











#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include <string>
#include <map>
#include <iostream>
#include "flirBFS.h"
#include <json/json.h>
#include <json/value.h>

#include <chrono>  // chrono::system_clock
#include <ctime>   // localtime
#include <sstream> // stringstream
#include <iomanip> // put_time

#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <thread>

using namespace std;

class ParamListener : public rclcpp::Node, public BFSMulti
{
public:
    ParamListener() 
        : Node("param_listener"), 
          image_transport_(this) // Initialize image transport for this node
    {
        initSubscribers();
        createPublishers();
        createTimers();

        // Other initialization
        cam_count = 0;
        frame_rate = 10.0;
        sleep_time = 100;
    }

    ~ParamListener()
    {
        // Clean up resources if necessary
    }

private:
    rclcpp::Node::SharedPtr nh_;
    std::map<string, string> params_rev;
    SystemPtr system;
    CameraList camList;
    std::map<string, std::map<string, string>> paras_cams;
    std::shared_ptr<BFSMulti> pCameras{new BFSMulti()}; 
    std::thread _threadWorker;

    rclcpp::Time ros_time; 
    double rostimeSec;
    string str;

    // ROS2 Subscribers
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr connect_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cont_trigger_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr stop_cont_trigger_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr disconnect_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr frame_rate_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr capture_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr exposure_time_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr start_preview_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr stop_preview_sub;

    image_transport::ImageTransport image_transport_;
    image_transport::Publisher pub, pub1, pub2, pub3, pub4, pub5, pub6;
    std::vector<image_transport::Publisher> pubvector;

    int cam_count;
    double frame_rate;
    int sleep_time;

    string CurrentDayFolder;
    string dataPath = "/media/nvidia/Data2/"; // Example path

};