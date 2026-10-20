"""
Previsualiza la trayectoria del vuelo real sin Gazebo ni dron.
Usa el mismo trajectory_planner y los mismos parámetros que real_flight.

    ros2 launch control_bebop preview_trajectory.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('control_bebop')
    cfg = os.path.join(pkg, 'config', 'real_drone.yaml')
    return LaunchDescription([
        Node(package='control_bebop', executable='trajectory_planner',
             name='trajectory_controller', parameters=[cfg], output='screen'),
        Node(package='control_bebop', executable='trajectory_preview',
             name='trajectory_preview', parameters=[cfg, {'show_plot': False}],
             output='screen'),
        Node(package='control_bebop', executable='plot_3d',
             name='plot_3d', parameters=[cfg], output='screen'),
    ])

