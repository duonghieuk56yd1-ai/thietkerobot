"""robot_state_publisher cho AMR (dùng chung cho RViz2 độc lập và Gazebo)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gazebo = LaunchConfiguration('use_gazebo')

    xacro_file = PathJoinSubstitution(
        [FindPackageShare('amr_description'), 'urdf', 'amr.urdf.xacro'])
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_gazebo:=', use_gazebo]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Dùng đồng hồ mô phỏng /clock'),
        DeclareLaunchArgument(
            'use_gazebo', default_value='true',
            description='Chèn các thẻ <gazebo> (plugin, cảm biến) vào URDF'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
                'publish_frequency': 50.0,
            }],
        ),
    ])
