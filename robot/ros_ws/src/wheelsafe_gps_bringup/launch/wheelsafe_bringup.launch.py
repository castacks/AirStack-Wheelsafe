import os
import math

import ament_index_python

import launch
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node, SetRemap

from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, TextSubstitution

# Path to the launch files and directories that we will use
_MICROSTRAIN_LAUNCH_FILE = os.path.join(
    ament_index_python.packages.get_package_share_directory(
        "microstrain_inertial_driver"
    ),
    "launch",
    "microstrain_launch.py",
)
_GQ7_PARAMS_FILE = os.path.join(
    ament_index_python.packages.get_package_share_directory("wheelsafe_gps_bringup"),
    "config",
    "gq7.yaml",
)
_RVIZ_DISPLAY_FILE = os.path.join(
    ament_index_python.packages.get_package_share_directory("wheelsafe_gps_bringup"),
    "config",
    "display.rviz",
)

joy_config_filepath = os.path.join(
    ament_index_python.packages.get_package_share_directory("wheelsafe_gps_bringup"),
    "config",
    "logitech.yaml",
)

print(f"Using params file at {_GQ7_PARAMS_FILE}")
print(f"Using rviz file at {_RVIZ_DISPLAY_FILE}")
print(f"Using joy config file at {joy_config_filepath}")



def generate_launch_description():

    joy_dev = launch.substitutions.LaunchConfiguration('joy_dev')
    publish_stamped_twist = launch.substitutions.LaunchConfiguration('publish_stamped_twist')
    config_filepath = launch.substitutions.LaunchConfiguration('config_filepath')
    deadzone = launch.substitutions.LaunchConfiguration('deadzone')

    return LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument('joy_vel', default_value='cmd_vel'),
            launch.actions.DeclareLaunchArgument('joy_config', default_value='ps3'),
            launch.actions.DeclareLaunchArgument('joy_dev', default_value='0'),
            launch.actions.DeclareLaunchArgument('publish_stamped_twist', default_value='false'),
            launch.actions.DeclareLaunchArgument('config_filepath', default_value=joy_config_filepath),
            launch.actions.DeclareLaunchArgument('deadzone', default_value='0.3'),
            # Microstrain node
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(_MICROSTRAIN_LAUNCH_FILE),
                launch_arguments={
                    "configure": "true",
                    "activate": "true",
                    "params_file": _GQ7_PARAMS_FILE,
                    "namespace": "/",
                }.items(),
            ),
            # Publish a static transform for where the GQ7 is mounted on base_link.
            # Unless the GQ7 is mounted exactly one meter above base_link, you should
            # change this to be accurate to your setup
            # NOTE: This is the transform FROM the base_link frame TO the gq7_link
            # frame. I.e., relative orientation from base_link frame to gq7_link frame,
            # and subtracting the position of the gq7_link frame expressed in the
            # baes_link frame.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen",
                arguments=[
                    "--x",
                    "-0.17",
                    "--y",
                    "-0.22",
                    "--z",
                    "0",
                    "--roll",
                    "3.14",  # 180
                    "--pitch",
                    "0",
                    "--yaw",
                    "1.57",  # 90
                    "--frame-id",
                    "base_link",
                    "--child-frame-id",
                    "gq7_link",
                ],
            ),
            # TODO: Add launch action for bringing up the copley node here.
            Node(
                package="copley_ros",
                executable="copley_node",
                output="screen",
                remappings=[],  # Add remappings here
                parameters=[],  # Add parameters here
            ),
            Node(
                package='joy_linux', executable='joy_linux_node', name='joy_node',
                parameters=[{
                    'device_id': joy_dev,
                    'deadzone': deadzone,
                    'autorepeat_rate': 20.0,
            }]),
            Node(
                package='teleop_twist_joy', executable='teleop_node',
                name='teleop_twist_joy_node',
                parameters=[config_filepath, {'publish_stamped_twist': publish_stamped_twist}],
                remappings={('/cmd_vel', launch.substitutions.LaunchConfiguration('joy_vel'))},
            ),
        ]
    )
