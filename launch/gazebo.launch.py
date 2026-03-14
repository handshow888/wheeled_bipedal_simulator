#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包路径
    pkg_share = get_package_share_directory('wheeled_bipedal_simulator')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    
    # 启动Gazebo（使用gazebo.launch.py）
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
        ),
        # launch_arguments={'verbose': 'true'}.items()
    )

    # 静态TF发布器
    # static_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    #     name='tf_footprint_base'
    # )

    # URDF文件路径
    urdf_file = os.path.join(pkg_share, 'urdf', 'wheeled_bipedal_robot.urdf')
    
    # 读取URDF文件内容
    with open(urdf_file, 'r', encoding='utf-8') as infp:
        robot_desc = infp.read()

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
            '-file', urdf_file,
            '-x', '0',
            '-y', '0',
            '-z', '0.5'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        # static_tf,
        robot_state_publisher,
        spawn_entity,
    ])