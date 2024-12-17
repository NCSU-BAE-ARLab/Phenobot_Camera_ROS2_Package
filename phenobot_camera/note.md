This is a ROS2 package for PhenoStreo camera developed at AR Lab, NCSU (Dr. Lirong Xiang).
This is migrated from its original ROS1 package.

Currently tested with ROS2 Humble, Ubuntu 22.04 on a Jetson Orin device.

#Dependencies packages:
Serieal 1.2.1
(Included as a sibling folder)

#Instructions to Build the package:
Create a colcon workspace
Download and put the phenobot_camera and serial folders into 'your_colcon_ws/src' 
Build the package:
colcon build

#Uses
1)First follow the camera setup instructions and make sure all the connections are appropiate.

2) Make sure you have added the setup.bash in your ~/.bashrc file

3) open a terminal and run: (This will start the main camera node)
ros2 launch phenobot_camera start_camera.py
	
4) In a separate terminal window, run (to capture the images):
 ros2 run phenobot_camera connect_camera.py
 
 
 That's it!
 
 #Note
 The package is working fine. Please consider modifying it to suits your needs. 

	
