from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml



def generate_launch_description():
    elevation_map_param_file = os.path.join(
        get_package_share_directory("elevation_mapping_ros2"), 
        'config',
        'elevation_map.param.yaml')
    
    with open(elevation_map_param_file, 'r') as f:
        data = yaml.safe_load(f)
        params = data['/**']['ros__prameters']
    
    
    elevation_mapping_node = Node(
        package="elevation_mapping_ros2",
        name="elevation_mapping_ros2_node",
        executable="elevation_mapping_ros2_node",
        parameters=[params], 
    )
    return LaunchDescription([
        elevation_mapping_node,
    ])