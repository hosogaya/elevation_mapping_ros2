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
    elevation_map_param_file = os.path.join(
        get_package_share_directory("elevation_mapping_ros2"), 
        'config',
        'elevation_map.param.yaml')
    
    topic_name_file = os.path.join(
        get_package_share_directory("elevation_mapping_ros2"), 
        'config', 
        'topic_name.yaml')
    post_processing_file = os.path.join(
        get_package_share_directory('elevation_mapping_ros2'), 
        'config', 
        'post_processing.param.yaml'
    )

    visualization_file = os.path.join(
        get_package_share_directory("elevation_mapping_ros2"),
        'config',
        'visualization.yaml' 
    )
    
    params = read_yaml(elevation_map_param_file)
    params_post_processing = read_yaml(post_processing_file)
    topic_name = read_yaml(topic_name_file)
    
    map_container = ComposableNodeContainer(
        name='map_container',
        namespace='',
        package='rclcpp_components', 
        executable='component_container', 
        composable_node_descriptions=[
            ComposableNode(
                package='elevation_mapping_ros2', 
                plugin='elevation_mapping::ElevationMapping', 
                name='elevation_mapping_node', 
                remappings=[("input/point_cloud", topic_name["input"]["point_cloud"]), 
                    ("input/pose", topic_name["input"]["pose_covariance"]), 
                    ("output/raw_map", topic_name["output"]["raw_map"])],
                parameters=[params]
            )
        ], 
        output='screen'
    )
    
    post_processing_container = ComposableNodeContainer(
        name='post_processing_container',
        namespace='',
        package='rclcpp_components', 
        executable='component_container', 
        composable_node_descriptions=[
            ComposableNode(
                package='elevation_mapping_ros2', 
                plugin='elevation_mapping::PostProcessor', 
                name='post_processor_node', 
                remappings=[("output/grid_map", "filtered_map"),
                    ("input/grid_map", topic_name["output"]["raw_map"])],
                parameters=[params_post_processing], 
            )
        ], 
        output='screen'
    )
            
    # elevation_mapping_node = Node(
    #     package="elevation_mapping_ros2",
    #     name="elevation_mapping_ros2_node",
    #     executable="elevation_mapping_ros2_node",
    #     parameters=[params], 
    #     remappings=[("input/point_cloud", topic_name["input"]["point_cloud"]), 
    #                 ("input/pose", topic_name["input"]["pose_covariance"]), 
    #                 ("output/raw_map", topic_name["output"]["raw_map"])], 
    #     arguments=['--ros-args', '--log-level', 'INFO'], 
    #     output = 'screen'
    # )

    # elevation_mapping_composition = Node(
    #     package='elevation_mapping_ros2', 
    #     executable='elevation_mapping_ros2_composition', 
    #     name='elevation_mapping_ros2_composition', 
    #     parameters=[params, params_post_processing], 
    #     remappings=[("input/point_cloud", topic_name["input"]["point_cloud"]), 
    #                 ("input/pose", topic_name["input"]["pose_covariance"]), 
    #                 ("output/raw_map", topic_name["output"]["raw_map"]), 
    #                 ("output/grid_map", "filtered_map"),
    #                 ("input/grid_map", topic_name["output"]["raw_map"]),
    #                 ],
    #     arguments=['--ros-args', '--log-level', 'INFO'], 
    #     output = 'screen'
    # )
    
    elevation_raw_map_visualization = Node(
        package='grid_map_visualization',
        executable='grid_map_visualization',
        name='grid_map_visualization',
        output='screen',
        parameters=[visualization_file]
    )
    
    return LaunchDescription([
        # elevation_mapping_composition,
        map_container, 
        post_processing_container,
        elevation_raw_map_visualization, 
    ])