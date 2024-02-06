from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from ament_index_python import get_package_share_directory
import os
import yaml

def read_yaml(file_path: str):
    with open(file_path, 'r') as f:
        data = yaml.safe_load(f)
        params = data['elevation_mapping']['ros__parameters']
    return params

def launch_setup(context, *args, **kwargs):
    param = read_yaml(LaunchConfiguration("param_file_path").perform(context))
    topic_name = read_yaml(LaunchConfiguration("remapping_file_path").perform(context))

    load_composable_nodes = LoadComposableNodes(
        target_container=LaunchConfiguration("container_name"),
        composable_node_descriptions=[
            ComposableNode(
                package="elevation_mapping_ros2",
                plugin="elevation_mapping::ElevationMapping",
                name="elevatoin_mapping",
                extra_arguments=[{"use_intra_process_comms": True}],
                parameters=[param], 
                remappings=[("elevation_mapping/input/point_cloud", topic_name["input"]["point_cloud"]), 
                    ("elevation_mapping/input/pose", topic_name["input"]["pose_covariance"]), 
                    ("elevation_mapping/output/raw_map", topic_name["output"]["raw_map"]), 
                    ],
            ),
        ],
    )

    return [
        load_composable_nodes
    ]

def generate_launch_description():
    arg_container_name = DeclareLaunchArgument(
        "container_name", default_value=TextSubstitution(text="my_container")
    )
    arg_param_file_path = DeclareLaunchArgument(
        "param_file_path", default_value=TextSubstitution(
            text=os.path.join(
                get_package_share_directory("elevation_mapping_ros2"), 
                'config', 'elevation_mapping.param.yaml'
            )
        )
    )

    arg_remapping_file_path = DeclareLaunchArgument(
        "remapping_file_path", default_value=TextSubstitution(
            text=os.path.join(
                get_package_share_directory("elevation_mapping_ros2"), 
                'config/topic_name.yaml'
            )
        )
    )


    return LaunchDescription([
        arg_container_name,
        arg_param_file_path, 
        arg_remapping_file_path,
        # load_composable_nodes, 
        OpaqueFunction(function=launch_setup)
    ])