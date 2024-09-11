from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value='/home/seunghyun/omo_ws/src/vfh_planner/config/vfh.yaml',
            description='Full path to the parameter file to use for the node'
        ),
        Node(
            package='vfh_planner',
            executable='vfh_node',
            name='vfh_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')]
        )
    ])
