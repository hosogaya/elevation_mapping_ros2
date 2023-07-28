from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def read_yaml(file_path):
    with open(file_path, 'r') as f:
        data = yaml.safe_load(f)
        params = data['/**']['ros__parameters']
    return params

def generate_launch_description():
    elevation_map_param_file = os.path.join(
        get_package_share_directory("elevation_mapping_ros2"), 
        'config',
        'elevation_map.param.yaml')
    
    topic_name_file = os.path.join(
        get_package_share_directory("elevation_mapping_ros2"), 
        'config', 
        'topic_name.yaml')
    
    params = read_yaml(elevation_map_param_file)
    topic_name = read_yaml(topic_name_file)
        
    elevation_mapping_node = Node(
        package="elevation_mapping_ros2",
        name="elevation_mapping_ros2_node",
        executable="elevation_mapping_ros2_node",
        parameters=[params], 
        remappings=[("input/point_cloud", topic_name["input"]["point_cloud"]), 
                    ("input/pose", topic_name["input"]["pose_covariance"]), 
                    ("output/raw_map", topic_name["output"]["raw_map"])], 
        arguments=['--ros-args', '--log-level', 'INFO'], 
        output = 'screen'
    )
    return LaunchDescription([
        elevation_mapping_node,
    ])