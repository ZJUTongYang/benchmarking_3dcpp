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

    config_filename_arg = DeclareLaunchArgument(
        'config_filename',
        default_value=os.path.join(get_package_share_directory('benchmarking_3dcpp'), 'config', 'config.yaml'),
    )

    benchmarking_3dcpp_node = Node(
        package='benchmarking_3dcpp',
        executable='benchmarking_3dcpp_node',
        name='benchmarking_3dcpp_node',
        parameters=[
            {'config_filename': LaunchConfiguration('config_filename')}
        ],
        output='screen'
    )

    return LaunchDescription([
        config_filename_arg,
        nuc_node,
        benchmarking_3dcpp_node
    ])

