"""
Launch file principal para vuelo de Bebop 2.

Lanza:
  1. Gazebo Sim con mundo bebop1.sdf
  2. Bridge ROS ↔ Gazebo
  3. Pose inicial del dron
  4. Flight Manager (máquina de estados)
  5. Joy node + Joystick Interface (PS5, condicional)
  6. Control PID
  7. Planificador de trayectoria
  8. Visor de odometría
  9. Nodo de video

Uso:
  # Con joystick (default):
  ros2 launch control_bebop main_flight.launch.py

  # Sin joystick (usar control_panel en terminal aparte):
  ros2 launch control_bebop main_flight.launch.py use_joystick:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ros_gz_bridge.actions import RosGzBridge
from launch.actions import ExecuteProcess
from datetime import datetime 

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_bebop_demo = get_package_share_directory('bebop_demo')
    pkg_control = get_package_share_directory('control_bebop')

    #Ruta guardado 
    bag_base_dir=os.path.expanduser('~/ros2_ws/src/control_bebop/bags')
    timestamp=datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_path = os.path.join(bag_base_dir, f'vuelo_data_{timestamp}')


    # ── Argumentos de launch ──
    use_joy_arg = DeclareLaunchArgument(
        'use_joystick', default_value='true',
        description='Habilitar control PS5 DualSense'
    )

    # ── 1. Gazebo Sim ──
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r bebop1.sdf'}.items(),
    )

    # ── 2. Bridge ROS ↔ Gazebo ──
    ros_gz_bridge = RosGzBridge(
        bridge_name='ros_gz_bridge',
        config_file=os.path.join(pkg_bebop_demo, 'config', 'bebop1.yaml'),
    )

    # ── 3. Pose inicial del dron (centro de la arena) ──
    set_pose = Node(
        package='bebop_demo',
        executable='set_pose',
        name='set_pose',
        output='screen',
        parameters=[
            {'robot_names': '["bebop1"]'},
            {'initial_conditions': '[[0.0, 0.0, 0.0, 0.0]]'}
        ]
    )

    # ── 4. Flight Manager (máquina de estados) ──
    flight_manager = Node(
        package='control_bebop',
        executable='flight_manager',
        name='flight_manager',
        output='screen',
        ros_arguments=['--log-level', 'WARN'],
        parameters=[
            {'takeoff_height': 1.0},
            {'arena_half_size': 1.8},
            {'safety_margin': 0.15},
            {'auto_land_after_orbit': True},
            {'auto_land_on_home': True},
            {'deadman_timeout': 0.5},
            {'landing_rate': 0.3},
            {'position_tolerance': 0.15},
            {'touchdown_z': 0.08},
            {'frequency': 20.0},
            {'home_x': 0.0},
            {'home_y': 0.0},
        ],
    )

    # ── 5a. Joy node (ROS2 joy package) ──
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[
            {'device_id': 0},
            {'deadzone': 0.05},
            {'autorepeat_rate': 20.0},
        ],
        condition=IfCondition(LaunchConfiguration('use_joystick')),
    )

    # ── 5b. Interfaz del joystick PS5 ──
    joystick_interface = Node(
        package='control_bebop',
        executable='joystick_interface',
        name='joystick_interface',
        output='screen',
        parameters=[
            os.path.join(pkg_control, 'config', 'ps5_dualsense.yaml'),
        ],
        condition=IfCondition(LaunchConfiguration('use_joystick')),
    )

    # ── 6. Controlador PID del dron ──
    controller = Node(
        package='control_bebop',
        executable='control_PID',
        name='control_PID',
        output='screen',
        parameters=[
            {'takeoff_height': 1.0},
            {'arena_limit': 1.8},
            {'max_vel_xy': 0.5},
            {'max_vel_z': 1.0},
            {'max_vel_yaw': 0.5},
            {'cmd_vel_topic': '/bebop1/cmd_vel'},
        ],
    )

    # ── 7. Planificador de trayectoria circular ──
    trajectory = Node(
        package='control_bebop',
        executable='trajectory_planner',
        name='trajectory_planner',
        output='screen',
        parameters=[
            {'target_x': 1.0},        # Centro X del círculo
            {'target_y': 0.0},         # Centro Y del círculo
            {'circle_radius': 0.15},    # Radio (m)
            {'circle_height': 1.0},    # Altura de vuelo
            {'circle_speed': 0.15},    # Velocidad angular (rad/s)
            {'arena_half_size': 1.8},  # Mitad del lado de la arena
            {'safety_margin': 0.15},   # Margen de seguridad
            # Posición exacta del bote de pastillas
            {'lookat_x': 1.0},
            {'lookat_y': 0.0},
            {'lookat_z': 1.025},
        ]
    )

    bag_record = ExecuteProcess(
        cmd=['ros2','bag','record','-o',bag_path,'/bebop1/odom','/goal','/flight_state', '/bebop1/cmd_vel'], #,'/bebop1/camera/image_raw'
        output='screen'
        )


    # ── 8. Visor de odometría ──
    gui_odometria = Node(
        package='control_bebop',
        executable='gui_odometria',
        name='gui_odometria',
        #output='screen',
        output='log',
        ros_arguments=['--log-level', 'FATAL'],  
        parameters=[
            {'frequency': 20.0},
        ]
    )

    # ── 9. Nodo de video ──
    video_node = Node(
        package='control_bebop',
        executable='nodo_video',
        name='nodo_video',
        output='screen',
        parameters=[
            {'camera_fps': 15.0},
        ],
    )

    return LaunchDescription([
        use_joy_arg,
        gz_sim,
        ros_gz_bridge,
        set_pose,
        flight_manager,
        joy_node,
        joystick_interface,
        controller,
        trajectory,
        bag_record,
        gui_odometria,
        video_node,
    ])
