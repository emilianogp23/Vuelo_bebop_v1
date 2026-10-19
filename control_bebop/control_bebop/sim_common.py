"""
Partes comunes de los dos launches de simulación.

    sim_flight_humble.launch.py  → laptop     (ROS 2 Humble + Gazebo Fortress)
    main_flight.launch.py        → escritorio (ROS 2 Jazzy  + Gazebo Harmonic)

Cada launch solo se encarga de lo que depende de su versión de Gazebo
(cómo arrancar el simulador y el puente ROS↔Gazebo). Los nodos del sistema,
sus parámetros (config/sim.yaml), la grabación del bag y las rutas viven aquí
para que las dos máquinas no se desincronicen.
"""

import getpass
import os
import tempfile
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from control_bebop.paths import data_dir

WORLD_NAME = 'bebop1.sdf'

# Tópicos de la simulación que se graban (/bebop1/odom lo publica odom_viewer)
SIM_BAG_TOPICS = [
    '/bebop1/pose', '/bebop1/odom', '/goal', '/goal_vel', '/flight_state',
    '/bebop1/cmd_vel', '/trajectory_done', '/joy', '/joy_cmd',
]


def declare_sim_arguments():
    """Argumentos iguales en los dos launches."""
    return [
        DeclareLaunchArgument('use_joystick', default_value='true',
                              description='Habilitar el mando PS5 DualSense'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Gazebo sin ventana (solo servidor)'),
        DeclareLaunchArgument('record_bag', default_value='true',
                              description='Grabar un bag en control_bebop/bags'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Abrir la ventana de odometría'),
    ]


def is_true(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ('true', '1', 'yes')


def world_path():
    return os.path.join(get_package_share_directory('bebop_gz'), 'worlds', WORLD_NAME)


def fortress_world_path():
    """Copia del mundo con los nombres de archivo de sistemas de Fortress.

    Harmonic llama a sus sistemas libgz-sim-*-system.so y Fortress
    libignition-gazebo-*-system.so. Los nombres de clase gz::sim::systems::*
    sí los acepta Fortress, así que solo cambia el atributo filename.
    """
    with open(world_path(), encoding='utf-8') as f:
        sdf = f.read()
    sdf = sdf.replace('filename="gz-sim-', 'filename="ignition-gazebo-')
    out_dir = os.path.join(tempfile.gettempdir(), f'bebop_gz_{getpass.getuser()}')
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, WORLD_NAME.replace('.sdf', '_fortress.sdf'))
    with open(out, 'w', encoding='utf-8') as f:
        f.write(sdf)
    return out


def add_resource_paths():
    """Agrega los modelos de objetos de control_bebop a las rutas de Gazebo.

    Así el mundo encuentra model://5_HTP y similares en ambas máquinas.
    Se modifica os.environ porque gz_sim.launch.py construye su entorno a
    partir de él.
    """
    meshes = os.path.join(get_package_share_directory('control_bebop'),
                          'resource', 'meshes')
    if not os.path.isdir(meshes):
        return
    for var in ('GZ_SIM_RESOURCE_PATH', 'IGN_GAZEBO_RESOURCE_PATH'):
        current = os.environ.get(var, '')
        if meshes not in current.split(os.pathsep):
            os.environ[var] = os.pathsep.join(p for p in (meshes, current) if p)


def distro_warning(expected, launch_name, alternative):
    """Aviso si el launch se corre en la máquina equivocada."""
    distro = os.environ.get('ROS_DISTRO', '')
    if distro and distro != expected:
        return [LogInfo(msg=(
            f'[{launch_name}] ADVERTENCIA: este launch es para ROS 2 {expected} '
            f'y estás en {distro}. Usa: ros2 launch control_bebop {alternative}'))]
    return []


def gz_args(world, context, extra=''):
    """Argumentos de gz/ign: -r arranca la simulación; -s sin ventana."""
    args = ['-r']
    if is_true(context, 'headless'):
        args += ['-s', '--headless-rendering']
    if extra:
        args.append(extra)
    args.append(world)
    return ' '.join(args)


def sim_nodes(context):
    """Nodos del sistema de control, iguales en las dos máquinas."""
    share = get_package_share_directory('control_bebop')
    sim_cfg = os.path.join(share, 'config', 'sim.yaml')
    joy_cfg = os.path.join(share, 'config', 'ps5_dualsense.yaml')
    use_joy = IfCondition(LaunchConfiguration('use_joystick'))

    actions = [
        Node(package='bebop_demo', executable='set_pose', name='set_pose',
             output='screen',
             parameters=[{'robot_names': '["bebop1"]',
                          'initial_conditions': '[[0.0, 0.0, 0.0, 0.0]]'}]),
        Node(package='control_bebop', executable='flight_manager',
             name='flight_manager', output='screen', parameters=[sim_cfg]),
        Node(package='joy', executable='joy_node', name='joy_node',
             output='screen', condition=use_joy,
             parameters=[{'device_id': 0, 'deadzone': 0.05,
                          'autorepeat_rate': 20.0}]),
        Node(package='control_bebop', executable='joystick_interface',
             name='joystick_interface', output='screen', condition=use_joy,
             parameters=[joy_cfg]),
        Node(package='control_bebop', executable='control_PID',
             name='control_PID', output='screen', parameters=[sim_cfg]),
        Node(package='control_bebop', executable='trajectory_planner',
             name='trajectory_planner', output='screen', parameters=[sim_cfg]),
        Node(package='control_bebop', executable='nodo_video',
             name='nodo_video', output='screen', parameters=[sim_cfg]),
        # Convierte /bebop1/pose en /bebop1/odom con velocidades
        Node(package='control_bebop', executable='odom_viewer',
             name='odom_viewer', output='log', parameters=[{'log_rate': 2.0}]),
    ]
    if is_true(context, 'gui'):
        actions.append(
            Node(package='control_bebop', executable='gui_odometria',
                 name='gui_odometria', output='log', parameters=[sim_cfg]))
    if is_true(context, 'record_bag'):
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        bag = os.path.join(data_dir('bags'), f'vuelo_sim_{stamp}')
        actions += [
            LogInfo(msg=f'Grabando bag en {bag}'),
            ExecuteProcess(cmd=['ros2', 'bag', 'record', '-s', 'sqlite3', '-o', bag] + SIM_BAG_TOPICS,
                           output='log'),
        ]
    return actions
