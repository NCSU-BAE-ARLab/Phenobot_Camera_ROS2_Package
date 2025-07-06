This is a ROS2 package for PhenoStreo camera developed at AR Lab, NCSU. This is migrated from its original ROS1 package by Prem Raj during his postdoctoral position at AR Lab

Currently tested with ROS2 Humble, Ubuntu 22.04 on a Jetson Orin device.

# Dependencies packages
Serieal 1.2.1 (Included as a sibling folder)

# Instructions to Build the package
1) Create a colcon workspace
2) Download and put the phenobot_camera and serial folders into 'your_colcon_ws/src'
3) Build the package
  ```bash
   colcon build
```

# Uses 
1) First follow the camera setup instructions and make sure all the connections are appropiate.

2) Make sure you have added the setup.bash in your ~/.bashrc file

3) open a terminal and run: (This will start the main camera node)
    ```bash
   ros2 launch phenobot_camera start_camera.py
    ```

5) In a separate terminal window, run (to capture the images):
  ```bash
   ros2 run phenobot_camera connect_camera.py
  ```

That's it!

# Note 
The package is working fine. Please consider modifying it to suits your needs.
