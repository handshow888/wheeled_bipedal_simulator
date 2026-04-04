#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    # 获取包路径
    description_pkg = get_package_share_directory('wheeled_bipedal_description')
    bringup_pkg = get_package_share_directory('wheeled_bipedal_bringup')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    world_file = LaunchConfiguration('world')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(bringup_pkg, 'rviz', 'gazebo_sim.rviz'),
        description='Full path to the RVIZ config file to use')  
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(bringup_pkg, 'worlds', 'RMUC2024_world.world'),
        # default_value="empty.world",
        description='Full path to the Gazebo world file to load')

    # 静态TF发布器
    # static_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
    #     name='tf_footprint_base'
    # )

    # URDF
    # urdf_file = os.path.join(description_pkg, 'urdf', 'wheeled_bipedal_robot.urdf')
    # with open(urdf_file, 'r', encoding='utf-8') as infp:
    #     robot_desc = infp.read()

    # xacro
    urdf_file = os.path.join(description_pkg, 'urdf', 'gazebo_ros2_control.xacro')
    robot_desc = Command(['xacro ', urdf_file])

    # 启动Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
                        #   'verbose': 'true',
                          'world': world_file,
                          }.items(),
    )

    # 机器人状态发布器（将URDF加载到参数服务器）
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'publish_frequency': 50.0
        }]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'wheeled_bipedal_robot',
            # '-file', urdf_file,
            '-topic', 'robot_description',
            '-x', '6.35',
            '-y', '7.6',
            '-z', '0.5'
        ],
        output='screen'
    )

    rviz_cmd = Node(
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
        wheeled_bipedal_controller_spawner,
        declare_use_rviz_cmd,
        declare_rviz_config_file_cmd,
        declare_world_cmd,
        rviz_cmd,
        gazebo,
        # static_tf,
        robot_state_publisher,
        spawn_entity,
        joint_state_broadcaster_spawner,
        imu_sensor_broadcaster_spawner,
        # diff_drive_controller_spawner,
        # effort_controller_spawner,
    ])