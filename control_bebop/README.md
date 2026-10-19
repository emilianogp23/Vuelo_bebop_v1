# Control Bebop
Paquete ROS 2 para controlar el Parrot Bebop 2, real y en simulación.

La documentación completa del proyecto está en [../DOCUMENTACION.md](../DOCUMENTACION.md).

## Vuelo real: notas de control

- **Altura.** La z de `/bebop/odom` es la velocidad vertical integrada y deriva:
  tras aterrizar marcó hasta +1.07 m. Con el parche del driver
  (`../patches/`), el bridge toma la z de `/bebop/altitude` (sonar + barómetro).
  `real_drone.yaml` usa `z_mode: native` (el Bebop mantiene su altura),
  `native_z_guard: true` y `z_geofence_enabled: true`. Sin el parche, pon los dos
  últimos en `false`.
- **Techo de seguridad.** El driver parchado manda `max_altitude` (2 m) al Bebop
  al conectar. Es un límite del firmware y no depende de la odometría.
- **Órbita atascada.** Si θ queda detenido `stall_timeout` s (3 s), el planner
  publica `/trajectory_stalled` y el flight_manager pasa a HOVER. R1 reanuda.
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

## Reconstrucción 3D

Del video de la órbita a una nube de puntos y una malla con COLMAP
(`sudo apt install colmap`). Detalles en [RECONSTRUCCION_3D.md](RECONSTRUCCION_3D.md).

```bash
python3 scripts/reconstruccion_3d.py videos/vuelo_YYYYMMDD_HHMMSS.avi
```
