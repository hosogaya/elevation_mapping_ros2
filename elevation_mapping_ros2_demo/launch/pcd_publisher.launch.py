import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # rviz_config_dir = os.path.join(get_package_share_directory(
        # 'pcd_demo'), 'config', 'config.rviz')
    # assert os.path.exists(rviz_config_dir)
    package_name = 'elevation_mapping_ros2_demo'
    ply_path = os.path.join(get_package_share_directory(
        package_name), 'resource', 'maze.ply')
    assert os.path.exists(ply_path)

    return LaunchDescription([
        # Node(package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', rviz_config_dir],
        #     output='screen'
        # ),
        Node(package=package_name,
            executable='pcd_publisher_node',
            name='pcd_publisher_node',
            output='screen',
            arguments=[ply_path],
            remappings=[('pcd', '/zedm/zed_node/point_cloud/cloud_registered')]
        ),
    ])

