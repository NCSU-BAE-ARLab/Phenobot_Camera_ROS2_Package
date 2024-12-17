#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "paramListener.h"
#include <boost/algorithm/string.hpp>
#include <map>
#include <errno.h>
#include <serial/serial.h> 

// #include <std_msgs/String.h> 

#include <std_msgs/msg/empty.hpp> 

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

#include <stdio.h>
#include <unistd.h>

#include "flirBFS.h"


void exposureTimeCallback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(rclcpp::get_logger("Main"), "I heard exposureTime: [%s]", msg->data.c_str());

}

void exposureAutoCallback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(rclcpp::get_logger("Main"),"I heard exposureAuto: [%s]", msg->data.c_str());
}

void triggerModeCallback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(rclcpp::get_logger("Main"),"I heard triggerMode: [%s]", msg->data.c_str());
}

void triggerSourceCallback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(rclcpp::get_logger("Main"),"I heard triggerSource: [%s]", msg->data.c_str());
}

void pixelFormatCallback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(rclcpp::get_logger("Main"),"I heard pixelFormat: [%s]", msg->data.c_str());
}

void gainAutoCallback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(rclcpp::get_logger("Main"),"I heard gainAuto: [%s]", msg->data.c_str());
}

void balanceWhiteAutoCallback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(rclcpp::get_logger("Main"),"I heard balanceWhiteAuto: [%s]", msg->data.c_str());
}

void gainCallback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(rclcpp::get_logger("Main"),"I heard gain: [%s]", msg->data.c_str());
}


void chatter1(const std_msgs::msg::Bool::SharedPtr msg)
{
  RCLCPP_INFO(rclcpp::get_logger("Main"),"I heard bool test : [%s]", msg->data ? "true" : "false");
}

void test() {
    std::cout << "Test the web server\n";

    // Initialize the ROS2 node
    rclcpp::init(0, nullptr);

    // Create a node
    auto node = std::make_shared<rclcpp::Node>("web_server_node");

    // Create subscribers for each topic
    auto sub1 = node->create_subscription<std_msgs::msg::String>(
        "exposureAuto", 10, exposureAutoCallback);
    
    auto sub2 = node->create_subscription<std_msgs::msg::String>(
        "exposureTime", 10, exposureTimeCallback);

    auto sub3 = node->create_subscription<std_msgs::msg::String>(
        "triggerMode", 10, triggerModeCallback);

    auto sub4 = node->create_subscription<std_msgs::msg::String>(
        "triggerSource", 10, triggerSourceCallback);

    auto sub5 = node->create_subscription<std_msgs::msg::String>(
        "pixelFormat", 10, pixelFormatCallback);

    auto sub6 = node->create_subscription<std_msgs::msg::String>(
        "gainAuto", 10, gainAutoCallback);

    auto sub7 = node->create_subscription<std_msgs::msg::String>(
        "gain", 10, gainCallback);

    auto sub8 = node->create_subscription<std_msgs::msg::String>(
        "balanceWhiteAuto", 10, balanceWhiteAutoCallback);

    // Spin to process the callbacks
    rclcpp::spin(node);

    // Shutdown ROS2
    rclcpp::shutdown();
}



int main(int argc, char **argv)
{

  // rclcpp::init(argc, argv, "camera_listener");

 

  // rclcpp::NodeHandle nh("~");

  rclcpp::init(argc, argv);
  // auto nh = rclcpp::Node::make_shared("camera_listener");
  // test();

  RCLCPP_INFO(rclcpp::get_logger("Main"),"main: instantiating an object of type Param_listener");
  // Param_listener param_listener("camera_listener");
  auto param_listener = std::make_shared<Param_listener>("camera_listener");


 // cout<<"params_rev.size(): "<<param_listener.params_rev.size()<<endl;

  // rclcpp::ExecutorOptions options;
  // options.num_threads = 4; 

  
//  rclcpp::spin();

  // rclcpp::executors::MultiThreadedExecutor spinner(options);
  

  // rclcpp::MultiThreadedSpinner spinner(4);
  // spinner.spin();

/*
  while(rclcpp::ok()){

      rclcpp::spinOnce();

      //cout<<"params_rev.size(): "<<param_listener.params_rev.size()<<endl;

      //break;
  }*/

}
