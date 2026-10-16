# Control Bebop
Paquete ROS 2 para controlar el Parrot Bebop 2, real y en simulación.

La documentación completa del proyecto está en [../DOCUMENTACION.md](../DOCUMENTACION.md).

## Vuelo real: notas de control

- **Altura.** La z de `ros2_bebop_driver` es la velocidad vertical integrada y deriva:
  tras aterrizar marcó entre −0.31 y +0.92 m. Por eso `real_drone.yaml` usa
  `z_mode: native` (el Bebop mantiene su altura), `native_z_guard: false` y
  `z_geofence_enabled: false`. La geocerca XY sigue activa.
- **Techo de seguridad.** Configura la altitud máxima del Bebop en 2 m desde
  FreeFlight. Es un límite del firmware y no depende de la odometría.
- **Ganancias XY.** El comando pitch/roll del Bebop es inclinación (≈aceleración) con
  retardo de ~0.6–0.8 s. `kp_xy`, `kd_xy` y `ff_gain_xy` se eligieron con modelos
  identificados de los bags. El término D usa la velocidad medida de `/odom_local`.
- **Giro.** El Bebop gira ~0.21 rad/s por unidad de comando. `circle_speed` y
  `max_yaw_rate` deben quedar por debajo de ~0.15 rad/s en `yaw_mode: lookat`.

Para revisar un vuelo:

```bash
source /opt/ros/humble/setup.bash
python3 scripts/analisis_bag.py bags/vuelo_real_YYYYMMDD_HHMMSS
```
