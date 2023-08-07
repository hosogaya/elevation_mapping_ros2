from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def read_yaml(file_path):
    with open(file_path, 'r') as f:
        data = yaml.safe_load(f)
        params = data['/**']['ros__parameters']
    return params

def generate_launch_description():
    plane_extractor_container = ComposableNodeContainer(
        name='plane_extractor_container',
        namespace='',
        package='rclcpp_components', 
        executable='component_container', 
        composable_node_descriptions=[
            ComposableNode(
                package='plane_extraction', 
                plugin='plane_extraction::PlaneExtractor', 
                name='plane_extractor_node',
                remappings=[("input/grid_map", "filtered_map")] 
            )
        ], 
        output='screen'
    )
    
    return LaunchDescription([
        plane_extractor_container,
    ])