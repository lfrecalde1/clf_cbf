from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os

def generate_launch_description():
    # Declare arguments
    launch_args = [ DeclareLaunchArgument('name', default_value='quadrotor'),
                    DeclareLaunchArgument('world_frame_id', default_value='world'),
                    DeclareLaunchArgument('camera_frame', default_value='camera'),
                ]

    # Use the LaunchConfiguration for each argument
    name = LaunchConfiguration('name')
    world_frame_id = LaunchConfiguration('world_frame_id')
    camera_frame = LaunchConfiguration('camera_frame')
    
    clf_cbf_node = ComposableNode(
        package='clf_cbf',  # Replace with your package name
        namespace= name,
        plugin='clf_cbf_node::ClfCbfNode',
        name='clf_cbf_nodelet',
        parameters=[
            {'world_frame_id': world_frame_id},
            {'quadrotor_name': name},
            {'camera_frame': camera_frame},
        ],
         remappings=[
            (['/', name, '/payload/clf'], ['/', name, '/payload/clf']),
            (['/', name, '/payload/cbf'], ['/', name, '/payload/cbf'])
        ],
    )
    clfcbf_container = ComposableNodeContainer(
        name='camera_container',
        namespace=name,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            clf_cbf_node,
        ],
        output='screen',
    )

    # Launch Description
    ld = LaunchDescription(launch_args)
    ld.add_action(
        GroupAction(
                actions=[clfcbf_container]
        ),
    )
    #ld.add_action(cbf_node)
    return ld