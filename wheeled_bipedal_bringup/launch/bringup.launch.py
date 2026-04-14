#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
def generate_launch_description():
    # 获取包路径
    description_pkg = get_package_share_directory('wheeled_bipedal_description')
    bringup_pkg = get_package_share_directory('wheeled_bipedal_bringup')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='False',
        description='Whether to start RVIZ')
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(bringup_pkg, 'rviz', 'gazebo_sim.rviz'),
        description='Full path to the RVIZ config file to use')

    declare_controller_config_cmd = DeclareLaunchArgument(
        'controller_config_file',
        default_value=os.path.join(bringup_pkg, 'config', 'wheeled_bipedal_controllers.yaml'),
        description='Full path to the ros2_control controller configuration YAML file'
    )

    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    controller_config_file = LaunchConfiguration('controller_config_file')

    # xacro
    urdf_file = os.path.join(description_pkg, 'urdf', 'real_ros2_control.xacro')
    robot_desc = Command(['xacro ', urdf_file])

    # 静态TF发布器
    # static_tf_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
    #     name='tf_footprint_base'
    # )

    # 机器人状态发布器（将URDF加载到参数服务器）
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'publish_frequency': 50.0
        }]
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        # arguments=['--ros-args', '-r', '~/robot_description:=/robot_description'],
        parameters=[
            {'robot_description': robot_desc},   # 暂时忽略弃用警告
            controller_config_file,
            ],
        output='screen',
    )


    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen')

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster']
    )

    imu_sensor_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['imu_sensor_broadcaster']
    )

    # diff_drive_controller_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['diff_drive_controller']
    # )

    effort_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['effort_controller']
    )

    wheeled_bipedal_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['wheeled_bipedal_controller']
    )

    return LaunchDescription([
        declare_use_rviz_cmd,
        declare_rviz_config_file_cmd,
        declare_controller_config_cmd,

        robot_state_publisher_node,
        controller_manager_node,
        # static_tf_node,

        joint_state_broadcaster_spawner,
        imu_sensor_broadcaster_spawner,
        # diff_drive_controller_spawner,
        # effort_controller_spawner,
        # wheeled_bipedal_controller_spawner,
        rviz_node,
    ])