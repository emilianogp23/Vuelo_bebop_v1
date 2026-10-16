# Parches a paquetes externos

## `ros2_bebop_driver_altitude.patch`

Parche para [jeremyfix/ros2_bebop_driver](https://github.com/jeremyfix/ros2_bebop_driver), hecho sobre el commit `d783e92`. Agrega dos cosas:

- **`/bebop/altitude`** (`std_msgs/Float64`, 15 Hz): la altura sobre el punto de despegue que mide el Bebop con sonar y barómetro (evento `PilotingStateAltitudeChanged`). Solo se publica si el dato tiene menos de 1 s. La z de `/bebop/odom` sigue siendo la velocidad integrada, que deriva.
- **Parámetro `max_altitude`** (por defecto 2.0 m): se envía al Bebop al conectar (`PilotingSettingsMaxAltitude`). Es el techo del firmware y reemplaza configurarlo en FreeFlight. El driver escribe en el log el valor aceptado y el rango permitido. Con un valor ≤ 0 se deja el que tenga guardado el dron.

Aplicar y compilar:

```bash
cd ~/ros2_ws/src/ros2_bebop_driver
git apply ~/ros2_ws/src/Vuelo_bebop_v1/patches/ros2_bebop_driver_altitude.patch
cd ~/ros2_ws && colcon build --packages-select ros2_bebop_driver
```

Comprobar:

```bash
ros2 launch ros2_bebop_driver bebop_node_launch.xml ip:=192.168.42.1 max_altitude:=2.0
# En el log: "MaxAltitude current=2.00 m (allowed ... )"
ros2 topic echo /bebop/altitude   # ~0 en el piso
```
