import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'control_bebop'

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
    ],
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
        ],
    },
)
