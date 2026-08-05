"""Full SAR demo: bridge + mission + detection + geotag + viz (+ RViz optional).

Arguments:
  rviz:=true|false   launch RViz with the SAR config (default false, headless env)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('sar_bringup')
    repo_root = os.path.abspath(os.path.join(bringup_share, '..', '..', '..', '..'))
    survey = os.path.join(bringup_share, 'launch', 'survey.launch.py')
    rviz_config = os.path.join(repo_root, 'config', 'rviz', 'sar.rviz')
    findings_csv = os.path.join(repo_root, 'output', 'findings.csv')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(survey)),
        Node(
            package='sar_viz',
            executable='telemetry_viz_node',
            name='telemetry_viz_node',
            parameters=[{'findings_csv': findings_csv}],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen',
        ),
    ])
