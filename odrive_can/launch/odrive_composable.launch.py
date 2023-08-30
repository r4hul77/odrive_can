import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, EmitEvent, OpaqueFunction, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode
import launch
import launch_ros
from launch_ros.events.lifecycle import ChangeState
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.actions import (DeclareLaunchArgument, EmitEvent,
                            RegisterEventHandler)
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

odrive_can_pkg = get_package_share_directory('odrive_can')
ros2_socketcan_pkg = get_package_share_directory('ros2_socketcan')
params_file = os.path.join(odrive_can_pkg, 'config', 'odrives_all_four_composable_params.yaml')

def create_launch_events(node_name, namespace):
    config_event = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_configure')),
    )

    activate_event = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=activate_event,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_activate')),
    )
    
    return [config_event, activate_event]


def generate_launch_description():
    
    compos_descs = []

    log_level = 'debug'
    
    for i in range(4):
        compos_descs.append(ComposableNode(
            package='odrive_can',
            plugin='odrive_node',
            name='odrive_{}'.format(i),
            parameters=[params_file],            
        ))
    # ros2_socketcan has two nodes that need to be launched together
    # one is the sender and the other is the receiver
    # both have their respective launch files named socket_can_sender.launch.py and socket_can_receiver.launch.py
    # the sender is launched here
    compos_descs.append(ComposableNode(
        package='ros2_socketcan',
        plugin='drivers::socketcan::SocketCanReceiverNode',
        name='socket_can_receiver',
        parameters=[params_file],
    ))
    
    compos_descs.append(ComposableNode(
        package='ros2_socketcan',
        plugin='drivers::socketcan::SocketCanSenderNode',
        name='socket_can_sender',
        parameters=[params_file],

    ))
    
    container = ComposableNodeContainer(
        name='odrive_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=compos_descs,
        output='screen',
        arguments=['--ros-args', '--log-level', 'debug'],
    )
    
    events = []
    #for node in compos_descs:
    #events.extend(create_launch_events('odrive_0', ''))
    

    lifecycle_manager =  Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager',
                    output='screen',
                    arguments=['--ros-args', '--log-level', log_level],
                    parameters = [params_file]
                    )
        
    
    return LaunchDescription([
        DeclareLaunchArgument('auto_activate', default_value='true'),
        DeclareLaunchArgument('auto_configure', default_value='true'),
        container,
        lifecycle_manager,
    ])