import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    
    stage_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('stage_ros2'),
                'launch',
                'demo.launch.py'
            )
        ),
        launch_arguments={
            'world': 'cave',
            'use_stamped_velocity': 'false'
        }.items()
    )


    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'base_frame': 'base_link'},
        ],
        remappings=[
            ('scan', 'base_scan'),
        ]
    )

    return LaunchDescription([
        stage_launch,
        slam_node,
    ])
