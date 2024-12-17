# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 3.14.4)
project(phenobot_camera)

#add_compile_options(-std=c++11)

#set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
#set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

SET(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})

## Find catkin and any catkin packages

find_package(OpenCV REQUIRED core highgui imgproc)
#find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg sensor_msgs image_transport cv_bridge serial) #cv_bridge

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
#find_package(ros2genmsg REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(serial REQUIRED)
find_package(image_transport REQUIRED)

#find_package(Boost REQUIRED filesystem system)
#find_package(OpenCV 4.0.0 REQUIRED)
find_package(OpenCV REQUIRED)
#find_package(rosidl_default_generators REQUIRED)


## Declare ROS messages and services


## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
ament_package()


#for json
file(GLOB INCLUDE_FILES "include/json/*.h")
install(FILES ${INCLUDE_FILES} DESTINATION include/${PROJECT_NAME}/json)

## Build talker and listener
#include_directories(include ${catkin_INCLUDE_DIRS})
#include_directories(${Boost_INCLUDE_DIRS} )
#include_directories ("/usr/include/opencv4/")
include_directories (${OpenCV_INCLUDE_DIRS})
include_directories(${serial_INCLUDE_DIRS})
include_directories(${image_transport_INCLUDE_DIRS})


include_directories("/opt/spinnaker/include")

#link_directories(${Boost_LIBRARY_DIRS} "/usr/include/jsoncpp" -lb64) 

link_directories("/opt/spinnaker/lib")

#add_executable(my_subscriber src/my_subscriber.cpp)
#target_link_libraries(my_subscriber ${catkin_LIBRARIES} ${OpenCV_LIBS})
#add_dependencies(my_subscriber beginner_tutorials_generate_messages_cpp)

set(SRC src/singleCamera.cpp src/jetsonGPIO.cpp src/flirBFS.cpp src/paramListener.cpp)
set(SRC_MAIN src/main.cpp)

add_executable(main ${SRC_MAIN} ${SRC})

#target_link_libraries(main -lpthread -lSpinnaker ${OpenCV_LIBS} ${Boost_LIBRARIES} ${std_msgs_TARGETS})

target_include_directories(main PUBLIC
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
   /opt/spinnaker
   ${OpenCV_INCLUDE_DIRS})
   
target_link_libraries(main PUBLIC
  ${OpenCV_LIBS}
  cv_bridge::cv_bridge
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  Spinnaker 
  ${serial_LIBRARIES}
  image_transport::image_transport)


install(TARGETS main
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

install(PROGRAMS
  scripts/connect_camera.py
  scripts/trigger_camera.py
  scripts/stop_trigger_camera.py
  scripts/test_rclpy.py
  DESTINATION lib/${PROJECT_NAME}
)

# %EndTag(FULLTEXT)%


