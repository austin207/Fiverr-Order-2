from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command

import os
from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    description = 'robot_description'
    bringup = 'robot_bringup'

    robot_description = os.path.join(get_package_share_path(description))
    gazebo_description = os.path.join(get_package_share_path('ros_gz_sim'))

    # rviz_config_path = os.path.join(get_package_share_path(description),
    #                                 'rviz', 'urdf_config.rviz')
    
    robot_brignup = os.path.join(get_package_share_path(bringup))
    robot_launch = os.path.join(robot_description, 'launch', 'robot_description.launch.py')
  
    controller_config = os.path.join(robot_brignup, 'config', 'ros2_control_config.yaml')
    gz_launch = os.path.join(gazebo_description, 'launch', 'gz_sim.launch.py')
    world = os.path.join(robot_brignup,'worlds', 'world2.sdf')

    ekf_config = os.path.join(robot_brignup, 'config', 'ekf.yaml')

    rviz2_config_path = os.path.join(robot_brignup,"config","rviz_config.rviz")
    robot_description_launch = IncludeLaunchDescription(launch_description_source = robot_launch)

    start_gazebo = IncludeLaunchDescription(launch_description_source = gz_launch,launch_arguments= {'gz_args': f"{world} -r",}.items())
    parameter_bridge_config = os.path.join(robot_brignup, 'config', 'gz_ros_topic_bridge.yaml')

    urdf_path = os.path.join(get_package_share_path('robot_description'),
                             'urdf', 'robot.xacro')
    
    robo_description_urdf = Command(['xacro ', urdf_path])

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_robot",                
        namespace="",                      
        arguments=[
            "-topic", "robot_description",  # no leading slash
            "-name", "robot",            # Gazebo model name (not TF prefix)
            "-allow_renaming", "false",      # don’t auto-rename if duplicate
            "-x", "-8.5",                     # X coordinate
            "-y", "9.5",                     # Y coordinate
            "-z", "0.5",                     # Z coordinate (height)
        ],
        output="screen",
        parameters=[{"use_sim_time": True}]
    )
    
    ros_gz_bridge = Node(
        package=  'ros_gz_bridge',
        executable="parameter_bridge",
        parameters=[
            {"config_file":parameter_bridge_config}]
    )
   
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz2_config_path]
    )

    nav2 = IncludeLaunchDescription(
        launch_description_source = os.path.join(robot_brignup,"launch","bringup_nav.py"),
        
    )

    slam = IncludeLaunchDescription(
        launch_description_source = os.path.join(robot_brignup,"launch","slam_launch.py"),
    )


    controller_manager = Node(
        package= "controller_manager",
        executable = "ros2_control_node",
        parameters=[
            {"use_sim_time": True},
            controller_config,
            {"robot_description": robo_description_urdf}],
    )

    ekf_node =  Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[ekf_config,{"use_sim_time":True}],
        )

    return LaunchDescription([
        robot_description_launch,
        start_gazebo,
        ros_gz_bridge,
        spawn_robot,
        slam,
        rviz2_node,
        ekf_node,
        nav2,
    ])