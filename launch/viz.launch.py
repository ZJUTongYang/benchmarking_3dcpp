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

    benchmarking_3dcpp_node = Node(
        package='benchmarking_3dcpp',
        executable='benchmarking_viz_node',
        parameters=[
        ],
        output='screen'
    )

    return LaunchDescription([
        benchmarking_3dcpp_node
    ])

