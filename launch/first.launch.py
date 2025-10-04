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

    scene_filename_arg = DeclareLaunchArgument(
        'scene_filename',
        default_value=os.path.join(get_package_share_directory('benchmarking_3dcpp'), 'scene', 'remeshed_saddle.stl'),
        description='Path to the surface to be covered.'
    )

    benchmarking_3dcpp_node = Node(
        package='benchmarking_3dcpp',
        executable='benchmarking_3dcpp_node',
        name='benchmarking_3dcpp_node',
        parameters=[
            {'scene_filename': LaunchConfiguration('scene_filename')}
        ],
        output='screen'
    )

    return LaunchDescription([
        scene_filename_arg,
        benchmarking_3dcpp_node
    ])

