"""Mô phỏng AMR vận chuyển hàng trên Gazebo Harmonic (gz-sim 8) + RViz2.

Ví dụ:
    ros2 launch amr_gazebo amr_gazebo.launch.py
    ros2 launch amr_gazebo amr_gazebo.launch.py world:=empty.sdf rviz:=false
    ros2 launch amr_gazebo amr_gazebo.launch.py spawn_cargo:=false
    # Đặt thùng hàng dưới sàn, trước mặt robot 1.5 m:
    ros2 launch amr_gazebo amr_gazebo.launch.py cargo_x:=1.5 cargo_z:=0.21
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
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    spawn_cargo = LaunchConfiguration('spawn_cargo')
    cargo_x = LaunchConfiguration('cargo_x')
    cargo_y = LaunchConfiguration('cargo_y')
    cargo_z = LaunchConfiguration('cargo_z')
    cargo_yaw = LaunchConfiguration('cargo_yaw')

    declare_args = [
        DeclareLaunchArgument(
            'world', default_value='warehouse.sdf',
            description='Tên file world trong amr_gazebo/worlds (warehouse.sdf | empty.sdf)'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='true: chỉ chạy gz server, không mở GUI Gazebo'),
        DeclareLaunchArgument('rviz', default_value='true', description='Mở RViz2'),
        DeclareLaunchArgument('x', default_value='0.0', description='Vị trí spawn x (m)'),
        DeclareLaunchArgument('y', default_value='0.0', description='Vị trí spawn y (m)'),
        DeclareLaunchArgument('z', default_value='0.01', description='Vị trí spawn z (m)'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Góc yaw spawn (rad)'),
        DeclareLaunchArgument(
            'spawn_cargo', default_value='true',
            description='Spawn thùng hàng thử nghiệm 0.4 m / 20 kg'),
        # Mặc định đặt thùng hàng nằm giữa khay: mặt băng 0.42 m + nửa cạnh 0.20 m + 0.02 m rơi tự do
        DeclareLaunchArgument('cargo_x', default_value=x, description='Thùng hàng: x (m)'),
        DeclareLaunchArgument('cargo_y', default_value=y, description='Thùng hàng: y (m)'),
        DeclareLaunchArgument('cargo_z', default_value='0.64', description='Thùng hàng: z (m)'),
        DeclareLaunchArgument('cargo_yaw', default_value=yaw, description='Thùng hàng: yaw (rad)'),
    ]

    # Cho Gazebo tìm model:// (giá kệ, trạm trao hàng, trạm sạc, thùng hàng)
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
        launch_arguments={'use_sim_time': 'true', 'use_gazebo': 'true'}.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_amr',
        output='screen',
        arguments=[
            '-name', 'amr',
            '-topic', 'robot_description',
            '-x', x, '-y', y, '-z', z, '-Y', yaw,
        ],
    )

    spawn_cargo_box = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_cargo_box',
        output='screen',
        condition=IfCondition(spawn_cargo),
        arguments=[
            '-name', 'cargo_box',
            '-file', os.path.join(pkg_gazebo, 'models', 'cargo_box', 'model.sdf'),
            '-x', cargo_x, '-y', cargo_y, '-z', cargo_z, '-Y', cargo_yaw,
        ],
    )

    # Spawn thùng hàng sau khi robot đã xuất hiện để thùng rơi đúng lên khay
    spawn_cargo_after_robot = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot, on_exit=[spawn_cargo_box]))

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{
            'config_file': os.path.join(pkg_gazebo, 'config', 'ros_gz_bridge.yaml'),
            'use_sim_time': True,
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', os.path.join(pkg_gazebo, 'rviz', 'amr_gazebo.rviz')],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription(
        declare_args + resource_paths + [
            gz_sim,
            robot_state_publisher,
            bridge,
            spawn_robot,
            spawn_cargo_after_robot,
            rviz,
        ])
