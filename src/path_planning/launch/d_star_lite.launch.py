from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'path_planning'

    rviz_config = PathJoinSubstitution([
        FindPackageShare(package_name),
        'rviz',
        'config.rviz'
    ])

    default_yaml_path = PathJoinSubstitution([
        FindPackageShare(package_name),
        'config',
        'moja_karta.yaml'
    ])

    yaml_arg = DeclareLaunchArgument(
        'yaml_filename',
        default_value=default_yaml_path,
        description='Path to map YAML file'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=rviz_config,
        description='Path to RViz config file'
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': LaunchConfiguration('yaml_filename')}]
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'autostart': True},
            {'node_names': ['map_server']}
        ]
    )

    dstar_node = Node(
        package='path_planning',
        executable='d_star_lite_node',
        name='d_star_lite_node',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')]
    )

    return LaunchDescription([
        yaml_arg,
        rviz_arg,
        map_server_node,
        lifecycle_manager_node,
        dstar_node,
        rviz_node
    ])
