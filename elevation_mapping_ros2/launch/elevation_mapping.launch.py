from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def read_yaml(file_path):
    with open(file_path, 'r') as f:
        data = yaml.safe_load(f)
        params = data['elevation_mapping']['ros__parameters']
    return params

def generate_launch_description():
    elevation_map_param_file_path = os.path.join(
        get_package_share_directory("elevation_mapping_ros2"), 
        'config',
        'elevation_mapping.param.yaml')
    
    remapping_name_file_path = os.path.join(
        get_package_share_directory("elevation_mapping_ros2"), 
        'config', 
        'topic_name.yaml')
    post_processing_file_path = os.path.join(
        get_package_share_directory('elevation_mapping_ros2'), 
        'config', 
        'post_processing.param.yaml'
    )
    
    container_name = "elevation_mapping_container"
    container = ComposableNodeContainer(
        name=container_name, 
        package="rclcpp_components",
        executable="component_container",
        namespace="",
        emulate_tty=True, # needed for display of logs, 
        output='screen'        
    )

    elevation_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory("elevation_mapping_ros2"), 
            '/launch/load_elevation_mapping.launch.py'
        ]), 
        launch_arguments={
            "container_name": TextSubstitution(text=container_name), 
            "param_file_path": TextSubstitution(text=elevation_map_param_file_path), 
            "remapping_file_path": TextSubstitution(text=remapping_name_file_path),
        }.items()
    )

    post_processor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory("elevation_mapping_ros2"), 
            '/launch/load_post_processing.launch.py'
        ]),
        launch_arguments={
            "container_name": TextSubstitution(text=container_name), 
            "param_file_path": TextSubstitution(text=post_processing_file_path), 
            "remapping_file_path": TextSubstitution(text=remapping_name_file_path),
        }.items()
    )
    
    return LaunchDescription([
        container,
        post_processor,
        elevation_mapping
    ])