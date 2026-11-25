"""
    Launch file
        1. Starts motor sliders window
        2. Trunk_shape (motor position and /joint angles )
        3. robot_state_publisher (URDF +joint angles )
        4. Rviz
    N.B on Raspberry pi (ros2 run trunk_control motor_sliders)
"""

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_share = FindPackageShare('trunk_description')

    # Run xacro on the model file and keep the result as one block of text.
    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([description_share, 'urdf', 'trunk.urdf.xacro'])]),
        value_type=str,
    )

    rviz_config = PathJoinSubstitution([description_share, 'rviz', 'trunk.rviz'])

    return LaunchDescription([
        Node(
            package='trunk_control',
            executable='motor_sliders',
            name='motor_sliders',
        ),
        Node(
            package='trunk_control',
            executable='trunk_shape',
            name='trunk_shape',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
        ),
    ])