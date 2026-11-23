"""Hiển thị độc lập AMR trên RViz2 (không cần Gazebo).

    ros2 launch amr_description display.launch.py            # có thanh trượt khớp
    ros2 launch amr_description display.launch.py gui:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('amr_description')
    gui = LaunchConfiguration('gui')
    rviz_config = LaunchConfiguration('rviz_config')

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, 'launch', 'rsp.launch.py'])),
        launch_arguments={'use_sim_time': 'false', 'use_gazebo': 'false'}.items(),
    )

    jsp_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(gui),
    )
    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(gui),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Mở joint_state_publisher_gui để kéo thử bánh xe / trụ nâng'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([pkg, 'rviz', 'display.rviz']),
            description='File cấu hình RViz2'),
        rsp,
        jsp_gui,
        jsp,
        rviz,
    ])
