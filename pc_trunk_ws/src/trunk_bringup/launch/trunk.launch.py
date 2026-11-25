"""Start the whole trunk with one command.

    ros2 launch trunk_bringup trunk.launch.py                    # model only, follows the sliders
    ros2 launch trunk_bringup trunk.launch.py sim:=true           # MuJoCo simulation
    ros2 launch trunk_bringup trunk.launch.py real_robot:=true    # the real motors
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    real_robot = LaunchConfiguration('real_robot')
    sim = LaunchConfiguration('sim')
    port = LaunchConfiguration('port')

    # True when neither the real robot nor the simulation is used.
    model_only = PythonExpression(
        ["'", real_robot, "' != 'true' and '", sim, "' != 'true'"])

    # The model and RViz2, from the description package, without its slider panel.
    display = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('trunk_description'), 'launch', 'display.launch.py')),
        launch_arguments={'gui': 'false'}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('real_robot', default_value='false',
                              description='true = talk to the Arduino and move the real motors'),
        DeclareLaunchArgument('sim', default_value='false',
                              description='true = run the MuJoCo simulation instead of the robot'),
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0',
                              description='Serial port of the Arduino Due'),

        display,

        # The window with three motor sliders.
        Node(package='trunk_control', executable='motor_sliders'),

        # Real robot: the bridge talks to the Arduino, and the shape
        # follows the positions the Arduino reports back.
        Node(package='trunk_control', executable='motor_bridge',
             parameters=[{'port': port}],
             output='screen',
             condition=IfCondition(real_robot)),
        Node(package='trunk_control', executable='trunk_shape',
             condition=IfCondition(real_robot)),

        # Simulation: MuJoCo plays the robot and publishes the shape itself.
        Node(package='trunk_sim', executable='sim',
             output='screen',
             condition=IfCondition(sim)),

        # Model only: the shape follows the slider commands directly.
        Node(package='trunk_control', executable='trunk_shape',
             remappings=[('trunk/motor_steps', 'trunk/motor_cmd')],
             condition=IfCondition(model_only)),
    ])