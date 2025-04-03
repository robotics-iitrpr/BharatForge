# Importing necessary libararies
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Opaque Function
def create_nodes(context, *args, **kwargs):
    rviz_file = os.path.join(get_package_share_directory('merge_map'), 'config', 'merge_map.rviz')
    bots = LaunchConfiguration('bots')
    return [
        # Rviz Node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_file],
            parameters=[{'use_sim_time': True}]
        ),
        # Merged map Node
        Node(
            package='merge_map',
            executable='merge_map',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['--bots', bots]
        )
    ]


def generate_launch_description():
    ld = LaunchDescription()
    bots = DeclareLaunchArgument(
        'bots',
        default_value='2',
    )
    ld.add_action(bots)
    bot_nodes = OpaqueFunction(function=create_nodes)
    ld.add_action(bot_nodes)
    return ld
