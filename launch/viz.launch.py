from launch_ros.substitutions import FindPackageShare 
from launch_ros.actions import Node 
from launch import LaunchDescription 
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, ExecuteProcess 
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument 
from launch.launch_description_sources import PythonLaunchDescriptionSource 
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution 
from ament_index_python.packages import get_package_share_directory 
import os 

def generate_launch_description():

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('benchmarking_3dcpp'),
        'rviz',
        'benchmarking_3dcpp.rviz'
    ])

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2', 
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    benchmarking_3dcpp_node = Node(
        package='benchmarking_3dcpp',
        executable='benchmarking_viz_node',
        parameters=[
        ],
        output='screen'
    )

    return LaunchDescription([
        rviz2_node,
        benchmarking_3dcpp_node
    ])

