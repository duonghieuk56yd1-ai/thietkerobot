"""Xem cơ cấu robot chia chọn khay lật (sorter_agv) trên Gazebo Harmonic + RViz2.

World trống, một robot và một bưu phẩm đặt sẵn trên khay, không có kịch bản đi lấy/trả hàng.

    ros2 launch amr_gazebo sorter_agv.launch.py
    ros2 run amr_gazebo amr_cmd.py tray right     # lật khay 30° thả hàng sang phải
    ros2 run amr_gazebo amr_cmd.py tray flat      # đưa khay về nằm ngang
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('amr_gazebo')
    pkg_description = get_package_share_directory('amr_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('rviz')
    spawn_parcel = LaunchConfiguration('spawn_parcel')

    declare_args = [
        DeclareLaunchArgument(
            'world', default_value='empty.sdf',
            description='Tên file world trong amr_gazebo/worlds'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='true: chỉ chạy gz server, không mở GUI Gazebo'),
        DeclareLaunchArgument('rviz', default_value='true', description='Mở RViz2'),
        DeclareLaunchArgument(
            'spawn_parcel', default_value='true',
            description='Đặt một bưu phẩm 0.30 × 0.22 × 0.12 m lên khay'),
    ]

    resource_paths = [
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(pkg_gazebo, 'models')),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(pkg_description)),
    ]

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': [
                PathJoinSubstitution([pkg_gazebo, 'worlds', world]),
                ' -r -v 3',
                PythonExpression(["' -s' if '", headless, "'.lower() == 'true' else ''"]),
            ],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, 'launch', 'rsp.launch.py')),
        launch_arguments={
            'model': 'sorter_agv', 'use_sim_time': 'true', 'use_gazebo': 'true'}.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_sorter_agv',
        output='screen',
        arguments=['-name', 'sorter_agv', '-topic', 'robot_description', '-z', '0.01'],
    )

    # Mặt khay ở z ≈ 0.288 m → tâm bưu phẩm 0.288 + 0.06 + 0.01 (rơi tự do 1 cm)
    spawn_parcel_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_parcel',
        output='screen',
        condition=IfCondition(spawn_parcel),
        arguments=[
            '-name', 'parcel',
            '-file', os.path.join(pkg_gazebo, 'models', 'parcel', 'model.sdf'),
            '-z', '0.36',
        ],
    )

    spawn_parcel_after_robot = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot, on_exit=[spawn_parcel_node]))

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{
            'config_file': os.path.join(pkg_gazebo, 'config', 'sorter_bridge.yaml'),
            'use_sim_time': True,
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', os.path.join(pkg_gazebo, 'rviz', 'sorter_agv.rviz')],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription(
        declare_args + resource_paths + [
            gz_sim,
            robot_state_publisher,
            bridge,
            spawn_robot,
            spawn_parcel_after_robot,
            rviz,
        ])
