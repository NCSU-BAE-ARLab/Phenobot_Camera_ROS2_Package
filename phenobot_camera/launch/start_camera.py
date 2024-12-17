from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the rosbridge_server launch file
    # rosbridge_launch_file = os.path.join(
    #     get_package_share_directory('rosbridge_server'),
    #     'launch',
    #     'rosbridge_websocket.launch.py'  # Ensure ROS 2 version exists
    # )

    return LaunchDescription([
        # Include rosbridge_websocket launch file with an argument
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(rosbridge_launch_file),
        #     launch_arguments={'port': '9090'}.items(),
        # ),

        # MJPEG server node
        # Node(
        #     package='mjpeg_server',
        #     executable='mjpeg_server',
        #     namespace='/robot',
        #     name='mjpeg_server',
        # ),

        # PhenoStereo1 node
        Node(
            package='phenobot_camera',
            executable='main',
            namespace='/robot',
            name='PhenoStereo1',
            output='screen',
            respawn=True,  # Equivalent to `respawn="true"`
        ),

        # # GPS node
        # Node(
        #     package='swiftnav_gps',
        #     executable='serialPortTest4.py',
        #     name='GPS_node',
        #     output='screen',
        # ),
    ])