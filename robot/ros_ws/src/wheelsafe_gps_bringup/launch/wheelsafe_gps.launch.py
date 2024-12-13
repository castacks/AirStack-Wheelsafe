import os
import math

import ament_index_python

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node, SetRemap

# Path to the launch files and directories that we will use
_MICROSTRAIN_LAUNCH_FILE = os.path.join(ament_index_python.packages.get_package_share_directory('microstrain_inertial_driver'), 'launch', 'microstrain_launch.py')
_GQ7_PARAMS_FILE = os.path.join(ament_index_python.packages.get_package_share_directory('wheelsafe_gps_bringup'), 'config', 'gq7.yaml')
_RVIZ_DISPLAY_FILE = os.path.join(ament_index_python.packages.get_package_share_directory('wheelsafe_gps_bringup'), 'config', 'display.rviz')

print(f"Using params file at {_GQ7_PARAMS_FILE}")
print(f"Using rviz file at {_RVIZ_DISPLAY_FILE}")

def generate_launch_description():
  return LaunchDescription([
    # Microstrain node
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(_MICROSTRAIN_LAUNCH_FILE),
      launch_arguments={
        'configure': 'true',
        'activate': 'true',
        'params_file': _GQ7_PARAMS_FILE,
        'namespace': '/',
      }.items()
    ),

    # Publish a static transform for where the GQ7 is mounted on base_link.
    # Unless the GQ7 is mounted exactly one meter above base_link, you should
    # change this to be accurate to your setup
    # NOTE: This is the transform FROM the base_link frame TO the gq7_link
    # frame. I.e., relative orientation from base_link frame to gq7_link frame,
    # and subtracting the position of the gq7_link frame expressed in the
    # baes_link frame.
    Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      output='screen',
      arguments=[
          "--x", "-0.17",
          "--y", "-0.22",
          "--z", "0",
          "--roll", "3.14", # 180
          "--pitch", "0",
          "--yaw", "1.57", # 90
          "--frame-id", "base_link",
          "--child-frame-id", "gq7_link"
        ]
    ),

    # # Run rviz to view the state of the application
    # Node(
    #   package='rviz2',
    #   executable='rviz2',
    #   output='screen',
    #   arguments=[
    #     '-d', _RVIZ_DISPLAY_FILE
    #   ]
    # ),
  ])