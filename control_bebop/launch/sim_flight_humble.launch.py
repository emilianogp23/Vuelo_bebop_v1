"""
Simulación del Bebop 2 en la LAPTOP: ROS 2 Humble + Gazebo Fortress.

El escritorio (ROS 2 Jazzy + Gazebo Harmonic) usa main_flight.launch.py.
Los nodos y parámetros comunes están en control_bebop/sim_common.py y
config/sim.yaml, así que las dos simulaciones se comportan igual.

Lo propio de Fortress:
    - El mundo bebop1.sdf se copia cambiando los nombres de archivo de los
      sistemas (gz-sim-* → ignition-gazebo-*).
    - El puente ROS↔Gazebo es ros_gz_bridge/parameter_bridge con config_file.

Uso:
    ros2 launch control_bebop sim_flight_humble.launch.py
    ros2 launch control_bebop sim_flight_humble.launch.py use_joystick:=false
    ros2 launch control_bebop sim_flight_humble.launch.py headless:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from control_bebop import sim_common


def _build(context):
    sim_common.add_resource_paths()
    world = sim_common.fortress_world_path()

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': sim_common.gz_args(world, context)}.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{'config_file': sim_common.bridge_config()}],
    )

    return (sim_common.distro_warning('humble', 'sim_flight_humble', 'main_flight.launch.py')
            + [gz_sim, bridge]
            + sim_common.sim_nodes(context))


def generate_launch_description():
    return LaunchDescription(
        sim_common.declare_sim_arguments() + [OpaqueFunction(function=_build)])
