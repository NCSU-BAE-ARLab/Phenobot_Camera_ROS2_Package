#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include <string>
#include <map>
#include <iostream>
#include "flirBFS.h"
#include <json/json.h>
//#include <jsoncpp/json/json.h>
#include <json/value.h>

#include <chrono>  // chrono::system_clock
#include <ctime>   // localtime
#include <sstream> // stringstream
#include <iomanip> // put_time
#include <fstream>
#include <iostream>
#include <memory>

//ros publish image
#include<image_transport/image_transport.h>
// #include "ffmpeg_image_transport/ffmpeg_image_transport.hpp"
#include<cv_bridge/cv_bridge.h>

using namespace std;

class Param_listener: public BFSMulti
{

public:
    // Param_listener(rclcpp::Node::SharedPtr node);
    Param_listener(const std::string &node_name);  

    ~Param_listener();
    
    std::map<string, string> params_rev;

	SystemPtr system;

	CameraList camList;
	
	std::map<string, std::map<string, string>> paras_cams;

	std::shared_ptr<BFSMulti> pCameras{new BFSMulti()}; 
	
	std::thread _threadWorker;

    rclcpp::Time ros_time; 
	double rostimeSec;
	
	string str;
	
	ofstream myfile;
private:

    // rclcpp::NodeHandle nh_;
    // rclcpp::NodeHandle nhi_;

    // rclcpp::Node("node_nh") nh_;
    // rclcpp::Node("node_nhi") nhi_;

    // auto nh_ = rclcpp::Node::make_shared("nh_node")
    // auto nhi_ = rclcpp::Node::make_shared("nhi_node")

    rclcpp::Node::SharedPtr nh_;
    rclcpp::Node::SharedPtr nhi_;

    // std::shared_ptr<rclcpp::Node> nh_;
    // std::shared_ptr<rclcpp::Node> nhi_;
    
    // rclcpp::Subscription::SharedPtr connect_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr connect_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cont_trigger_sub;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr  stop_cont_trigger_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr  disconnect_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr  frame_rate_sub;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr  capture_sub;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr  exposureTime_sub;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr  folderName_sub;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr  balanceWhiteAuto_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr  start_preview_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr  stop_preview_sub;

    void createDayFolder();

    string CurrentDayFolder;
    
    string CurrentRowFolder;

    string getCurrentDateStr();

	void initParamMap(); 
	void listen_param();
	void connectCallback(const std_msgs::msg::String::SharedPtr msg);
	void stopContTriggerCallback(const std_msgs::msg::String::SharedPtr msg);
	void contTriggerCallback(const std_msgs::msg::String::SharedPtr msg);
	void disconnectCallback(const std_msgs::msg::String::SharedPtr msg);
	void frameRateCallback(const std_msgs::msg::String::SharedPtr msg);
	void folderNameCallback(const std_msgs::msg::String::SharedPtr msg);
	void captureCallback(const std_msgs::msg::String::SharedPtr msg);
	void exposureTimeCallback(const std_msgs::msg::String::SharedPtr msg);
	void balanceWhiteAutoCallback(const std_msgs::msg::String::SharedPtr msg);
	void startPreviewCallback(const std_msgs::msg::String::SharedPtr msg);
    void stopPreviewCallback(const std_msgs::msg::String::SharedPtr msg);
	void preview();
	void preciseSleep(double seconds);

	string dataPath = "/home/ur5/prem/camera-data/"; // "/home/phenobot3/catkin_ws/Data/"   /home/nvidia/catkin_ws/Data/

	bool systemCreated = false;
	
	image_transport::Publisher pub, pub1, pub2, pub3, pub4, pub5, pub6, pub7, pub8, pub9, pub10;
    std::vector<image_transport::Publisher> pubvector;
    
    int cam_count;
    
    double frame_rate=10.f;

	int sleep_time=100;
};

