# Launch file to bring up RVIZ for visualizing + debugging GPS measurements.

import os
import math

import ament_index_python

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node, SetRemap

# Path to the launch files and directories that we will use
_RVIZ_DISPLAY_FILE = os.path.join(ament_index_python.packages.get_package_share_directory('wheelsafe_gps_bringup'), 'config', 'display.rviz')

def generate_launch_description():
  return LaunchDescription([

    # Run rviz to view the state of the application
    Node(
      package='rviz2',
      executable='rviz2',
      output='screen',
      arguments=[
        '-d', _RVIZ_DISPLAY_FILE
      ]
    ),
  ])