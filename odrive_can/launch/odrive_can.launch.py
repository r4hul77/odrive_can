import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, EmitEvent, OpaqueFunction, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode
import launch
import launch_ros
from launch_ros.events.lifecycle import ChangeState

from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.events import matches_action
import copy
from lifecycle_msgs.msg import Transition
import xacro


odrive_can_pkg = get_package_share_directory('odrive_can')
ros2_socketcan_pkg = get_package_share_directory('ros2_socketcan')
params_file = os.path.join(odrive_can_pkg, 'config', 'single_odrive_params.yaml')
def generate_launch_description():
    odrive_can_node = Node(
        package='odrive_can',
        executable='odrive_can_node',
        name='odrive_can_node_0',
        output='screen',
        parameters=[params_file]
    )
    # ros2_socketcan has two nodes that need to be launched together
    # one is the sender and the other is the receiver
    # both have their respective launch files named socket_can_sender.launch.py and socket_can_receiver.launch.py
    # the sender is launched here
    ros2_socketcan_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ros2_socketcan_pkg, '/launch/socket_can_sender.launch.py']),
        launch_arguments={
            'interface': 'can0',
            'timeout_sec': '0.01',
            'auto_configure': 'True',
            'auto_activate': 'True'
        }.items()
    )
    # the receiver is launched here
    ros2_socketcan_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ros2_socketcan_pkg, '/launch/socket_can_receiver.launch.py']),
        launch_arguments={
            'interface': 'can0',
            'timeout_sec': '0.01',
            'auto_configure': 'True',
            'auto_activate': 'True'
        }.items()
    )
    return LaunchDescription([
        odrive_can_node,
        ros2_socketcan_node
    ])