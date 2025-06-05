from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_coordinator', default_value='true'),

        Node(
            package='tree_navigation',
            executable='tree_detector',
            name='tree_detector',
            output='screen',
            parameters=[{'min_area': 500}, {'detection_rate': 30.0}]
        ),

        Node(
            package='tree_navigation',
            executable='brown_navigator',
            name='brown_navigator',
            output='screen',
            parameters=[{'forward_speed': 0.3}, {'turn_speed': 0.2}]
        ),

        Node(
            package='tree_navigation',
            executable='tree_navigation_coordinator',
            name='tree_navigation_coordinator',
            output='screen',
            condition=LaunchConfiguration('use_coordinator')
        ),
    ])
