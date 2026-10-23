"""
Simulación del Bebop 2 en el ESCRITORIO: ROS 2 Jazzy + Gazebo Harmonic.

La laptop (ROS 2 Humble + Gazebo Fortress) usa sim_flight_humble.launch.py.
Los nodos y parámetros comunes están en control_bebop/sim_common.py y
config/sim.yaml, así que las dos simulaciones se comportan igual.

Lo propio de Harmonic:
    - El mundo bebop1.sdf se carga tal cual.
    - El puente ROS↔Gazebo usa la acción RosGzBridge (no existe en Humble).

Uso:
    ros2 launch control_bebop main_flight.launch.py
    ros2 launch control_bebop main_flight.launch.py use_joystick:=false
    ros2 launch control_bebop main_flight.launch.py headless:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from control_bebop import sim_common


def _build(context):
    # Import aquí para que el archivo no falle al cargarse en Humble
    from ros_gz_bridge.actions import RosGzBridge

    sim_common.add_resource_paths()

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': sim_common.gz_args(sim_common.world_path(), context)}.items(),
    )

    bridge = RosGzBridge(
        bridge_name='ros_gz_bridge',
        config_file=sim_common.bridge_config(),
    )

    return (sim_common.distro_warning('jazzy', 'main_flight', 'sim_flight_humble.launch.py')
            + [gz_sim, bridge]
            + sim_common.sim_nodes(context))


def generate_launch_description():
    return LaunchDescription(
        sim_common.declare_sim_arguments() + [OpaqueFunction(function=_build)])
