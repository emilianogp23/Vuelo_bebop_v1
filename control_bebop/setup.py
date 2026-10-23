import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'control_bebop'


def _tree_data_files(root):
    """Instala una carpeta completa en share/, conservando su estructura.

    Se usa para resource/meshes: los modelos de objetos que carga Gazebo
    con model://<nombre> (ver config/sim.yaml y bebop_gz/worlds/bebop1.sdf).
    """
    entries = []
    for dirpath, _dirs, files in os.walk(root):
        if files:
            entries.append((os.path.join('share', package_name, dirpath),
                            [os.path.join(dirpath, f) for f in files]))
    return entries

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ] + _tree_data_files(os.path.join('resource', 'meshes')),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='emiliano',
    maintainer_email='emiliano@todo.todo',
    description='Control system for Bebop 2 drone with PS5 joystick support',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'trajectory_planner = control_bebop.trajectory_controller:main',
            'trajectory_preview = control_bebop.trajectory_preview:main',
            'odom_viewer = control_bebop.odom_viewer:main',
            'control_panel = control_bebop.control_panel:main',
            'control_PID = control_bebop.control_PID:main',
            'nodo_video = control_bebop.nodo_video:main',
            'flight_manager = control_bebop.flight_manager:main',
            'joystick_interface = control_bebop.joystick_interface:main',
            'real_drone_bridge = control_bebop.real_drone_bridge:main',
            'analisis_deriva = control_bebop.analisis_deriva:main',
            'gui_odometria = control_bebop.gui_odometria:main',
            'datos = control_bebop.datos:main',
            'muestra_odometria_gui = control_bebop.muestra_odometria_gui:main',
            'demo_safety_manager = control_bebop.demo_safety_manager:main',
            'control_camara = control_bebop.control_camara:main',
            'diagnostico_yolo = control_bebop.diagnostico_yolo:main',
            'aruco_localizer = control_bebop.aruco_localizer:main',
            'plot_3d = control_bebop.plot_3d:main',
            'sim_odometry = control_bebop.sim_odometry:main',
            'calibrar_camara = control_bebop.calibrar_camara:main',
        ],
    },
)
