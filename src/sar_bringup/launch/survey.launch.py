"""Full SAR survey: bridge + grid mission + detection + geotagging."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('sar_bringup')
    repo_root = os.path.abspath(os.path.join(bringup_share, '..', '..', '..', '..'))
    mission_params = os.path.join(repo_root, 'config', 'mission.yaml')
    bridge_config = os.path.join(bringup_share, 'config', 'bridge.yaml')
    output_dir = os.path.join(repo_root, 'output')

    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            parameters=[{'config_file': bridge_config}],
            output='screen',
        ),
        Node(
            package='sar_mission',
            executable='grid_mission_node',
            name='grid_mission_node',
            parameters=[mission_params],
            output='screen',
        ),
        Node(
            package='sar_perception',
            executable='detection_node',
            name='detection_node',
            output='screen',
        ),
        Node(
            package='sar_perception',
            executable='geotag_node',
            name='geotag_node',
            parameters=[{'output_dir': output_dir}],
            output='screen',
        ),
    ])
