# Vuelo_bebop_v1

Vuelo autónomo de un Parrot Bebop 2 con ROS 2 para reconstrucción 3D de objetos. El dron rodea el objeto en círculo mirando al centro y graba video. Ese video servirá para la reconstrucción y para planear un segundo vuelo con NBV (Next Best View).

**Toda la documentación está en [DOCUMENTACION.md](DOCUMENTACION.md):** arquitectura, nodos, máquina de estados, mando, parámetros, cómo volar en simulación y con el dron real, análisis de vuelos y problemas conocidos.

## Contenido

- `control_bebop/`: paquete principal con el control de vuelo, la trayectoria, el video y las herramientas.
- `bebop_ros/`: simulación en Gazebo, copia de juliordzcer/bebop_ros.

## Entornos

- Ubuntu 22.04 + ROS 2 Humble + Gazebo Fortress
- Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic

## Arranque rápido

```bash
cd ~/ros2_ws && colcon build --packages-select control_bebop && source install/setup.bash

# Previsualizar la trayectoria
ros2 launch control_bebop preview_trajectory.launch.py
# Simulación (Humble)
ros2 launch control_bebop sim_flight_humble.launch.py
# Dron real, con el driver ros2_bebop_driver corriendo en otra terminal
ros2 launch control_bebop real_flight.launch.py
```

Antes de volar el dron real, lee las secciones de seguridad y de vuelo real de la documentación.
