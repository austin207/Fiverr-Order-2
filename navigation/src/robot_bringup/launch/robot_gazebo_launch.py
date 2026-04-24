from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_path


def _derive_map_path(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
    map_yaml = world.replace('.sdf', '.yaml')
    return [SetLaunchConfiguration('derived_map', map_yaml)]


def _create_static_tf_node(context, *args, **kwargs):
    spawn_x = LaunchConfiguration('spawn_x').perform(context)
    spawn_y = LaunchConfiguration('spawn_y').perform(context)
    return [Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=[spawn_x, spawn_y, '0', '0', '0', '0', 'map', 'odom'],
        output='screen',
    )]


def generate_launch_description():
    description = 'robot_description'
    bringup = 'robot_bringup'

    robot_description = os.path.join(get_package_share_path(description))
    gazebo_description = os.path.join(get_package_share_path('ros_gz_sim'))

    robot_bringup = os.path.join(get_package_share_path(bringup))
    robot_launch = os.path.join(robot_description, 'launch', 'robot_description.launch.py')
    gz_launch = os.path.join(gazebo_description, 'launch', 'gz_sim.launch.py')

    ekf_config = os.path.join(robot_bringup, 'config', 'ekf.yaml')
    nav2_params = os.path.join(robot_bringup, 'config', 'nav2_params_rrt.yaml')
    rviz2_config_path = os.path.join(robot_bringup, 'config', 'rviz_config.rviz')
    parameter_bridge_config = os.path.join(robot_bringup, 'config', 'gz_ros_topic_bridge.yaml')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(robot_bringup, 'worlds', 'world2.sdf'),
        description='Full path to the .sdf world file'
    )
    spawn_x_arg = DeclareLaunchArgument('spawn_x', default_value='-8.5')
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='9.5')
    headless_arg = DeclareLaunchArgument('headless', default_value='true')
    gz_args_arg = DeclareLaunchArgument('gz_args', default_value='-r -s')
    derived_map_arg = DeclareLaunchArgument('derived_map', default_value='')

    derive_map_action = OpaqueFunction(function=_derive_map_path)

    robot_description_launch = IncludeLaunchDescription(launch_description_source=robot_launch)

    start_gazebo = IncludeLaunchDescription(
        launch_description_source=gz_launch,
        launch_arguments={
            'gz_args': [LaunchConfiguration('world'), ' ', LaunchConfiguration('gz_args')],
        }.items()
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        namespace='',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'robot',
            '-allow_renaming', 'false',
            '-x', LaunchConfiguration('spawn_x'),
            '-y', LaunchConfiguration('spawn_y'),
            '-z', '0.5',
        ],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': parameter_bridge_config}],
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz2_config_path],
        condition=UnlessCondition(LaunchConfiguration('headless')),
    )

    nav2 = IncludeLaunchDescription(
        launch_description_source=os.path.join(robot_bringup, 'launch', 'bringup_nav.py'),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[nav2_params,
                    {'use_sim_time': True,
                     'yaml_filename': LaunchConfiguration('derived_map')}],
    )

    localization_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': True,
                     'autostart': True,
                     'node_names': ['map_server']}],
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': True}],
    )

    return LaunchDescription([
        world_arg,
        spawn_x_arg,
        spawn_y_arg,
        headless_arg,
        gz_args_arg,
        derived_map_arg,
        derive_map_action,
        robot_description_launch,
        start_gazebo,
        ros_gz_bridge,
        spawn_robot,
        map_server,
        OpaqueFunction(function=_create_static_tf_node),
        localization_lifecycle,
        rviz2_node,
        ekf_node,
        nav2,
    ])