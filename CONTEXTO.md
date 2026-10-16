# Contexto del proyecto Vuelo_bebop_v1 (para futuros chats)

Última actualización: 16 oct 2026. Complementa a [DOCUMENTACION.md](DOCUMENTACION.md), que tiene el detalle. Aquí va lo esencial y el estado actual.

## 1. Qué es
Sistema ROS 2 para volar un Parrot Bebop 2 en una trayectoria circular alrededor de un objeto, con la cámara apuntando al centro, para grabar video y reconstruirlo en 3D (fotogrametría). Después vendrá NBV (Next Best View) y un segundo vuelo optimizado. Hoy solo existe la fase 1 (control de vuelo, órbita, video, simulación). La reconstrucción y el NBV no están en el repositorio.

Usuario: estudiante/investigador, trabaja en español, en un laboratorio. Dos computadoras:
- Laptop: Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress, `sim_flight_humble.launch.py`.
- Escritorio: Ubuntu 24.04, Jazzy, Harmonic, `main_flight.launch.py`.

Repositorio: `~/ros2_ws/src/Vuelo_bebop_v1`, rama `avance-laptop` (main es `main`). Git user: emilianogp23. Los bags (`control_bebop/bags/`) y videos no se suben a git.

## 2. Estructura
- `control_bebop/`: paquete principal (ament_python). Nodos en `control_bebop/`, launches, `config/real_drone.yaml`, `scripts/analisis_bag.py`, `scripts/replay_vuelo.py`, `test/`.
- `bebop_ros/`: simulación Gazebo (copia de juliordzcer/bebop_ros). Solo se usan `bebop_gz` y `bebop_demo`.
- `patches/`: parche al driver externo (altura real). Ver `patches/README.md`.
- Paquetes externos en `~/ros2_ws/src`: `ros2_bebop_driver` (jeremyfix, alfa, con cambios locales en CMakeLists.txt y el parche aplicado) y `ros2_parrot_arsdk`.

## 3. Arquitectura (vuelo real)
joy_node → joystick_interface → flight_manager (máquina de estados, única que publica `/flight_state`) → trajectory_planner (`/goal`, `/goal_vel`) → control_PID (10 Hz, `/bridge/cmd_vel_raw`) → real_drone_bridge (satura, geocerca, watchdog) → `/bebop/cmd_vel` del driver. El bridge convierte `/bebop/odom` al marco de despegue (`/bebop1/pose`, `/odom_local`).

Estados: 0 IDLE, 1 AUTOMATIC, 2 TAKING_OFF, 3 LANDING, 4 EMERGENCY, 5 HOVER, 6 MANUAL, 7 RECOVERY, 8 RETURN_HOME.

Marco de despegue: origen donde despega, x = frente del dron al despegar, y izquierda, z arriba. Centro del círculo por defecto en (1, 0), radio 0.5, ω 0.10 rad/s, `yaw_mode: lookat`.

Mando PS5: L2 es deadman (mantener todo el vuelo), × despega, R1 inicia órbita, □ alterna manual/auto, ○ aterriza, L1 home, △ aterrizaje inmediato, L1+R1+△ corta motores, Share FlatTrim.

Dato clave de dinámica: en el Bebop real el comando pitch/roll es inclinación (≈aceleración), con retardo 0.2 a 0.9 s. Con comando cero entra en hover nativo (sostiene posición con flujo óptico); con comando distinto de cero deja de hacerlo y deriva. Las ganancias de Gazebo no sirven en el real. Gira ~0.21 rad/s por unidad de comando.

## 4. Cambios hechos el 16 oct 2026 (sin commit todavía)
Tras un vuelo fallido (ver sección 5):
1. Driver parchado: publica `/bebop/altitude` (Float64, 15 Hz, altura medida por sonar+barómetro, solo si el dato tiene <1 s) y parámetro `max_altitude` (2.0 por defecto) que se manda al dron al conectar. Reemplaza FreeFlight.
2. Bridge: la z de `/bebop1/pose` sale de `/bebop/altitude` (`select_altitude` en `frames.py`; fallback a la z integrada con aviso cada 5 s).
3. `real_drone.yaml`: `z_geofence_enabled: true`, `native_z_guard: true`, `ki_xy: 0.05`, `int_limit_xy: 2.0` (el integral pasa de máx 0.015 a 0.10), `stall_timeout: 3.0`.
4. Órbita atascada: el planner publica `/trajectory_stalled` si θ queda detenido 3 s; el flight_manager pasa a HOVER; R1 con L2 reanuda.
5. `analisis_bag.py` reporta goal detenido, órbita atascada, z integrada contra altitud medida y posición final. `replay_vuelo.py` etiqueta la posición como "odometría".
6. Documentación y 18 pruebas (`python3 -m pytest test/ -q`, pasan).

## 5. Errores y problemas actuales
**Vuelo del 16 oct (bag `control_bebop/bags/vuelo_real_20261016_093834`):**
- No hizo el círculo: tras llegar al punto de entrada (0.5, 0) el dron derivó hacia atrás 0.5 a 1.5 m (estimación del usuario). El PID mandaba ~0.08 de pitch, el error pasó de `track_err_stop` (0.30 m) y el planner congeló θ (solo 90° en 47 s). El usuario tomó el mando manual y aterrizó en (0, 0).
- La odometría XY subestimó la deriva (~0.25 m) y al final marcaba (0.57, 0.65) estando en el origen: por eso el replay "aterriza en otro lado". El replay no tiene bug.
- z de 1.6 a 1.78 m falsa: nunca se mandó gaz>0; el dron estuvo a ~1 m. La z de `/bebop/odom` es velocidad vertical integrada y marcó +1.07 m en el piso. Resuelto en software con el parche, **pendiente de validar en el dron real**.

**Abiertos:**
- La posición XY viene de flujo óptico integrado y acumula error. No hay mocap en el laboratorio. Sin referencia externa, ningún ajuste del PID corrige una deriva que la odometría no ve. Mitigaciones: piso con textura, integral mayor, HOVER por órbita atascada. No se ha comprobado que basten.
- Los cambios de arriba no se han probado con el dron real ni en Gazebo (solo pruebas unitarias, compilación y un bridge con datos falsos).
- El escritorio necesita aplicar el parche al driver y recompilar; si no, poner `native_z_guard` y `z_geofence_enabled` en `false`.
- Magnetómetro en interiores degrada yaw y rotación de velocidades (en lookat el dron gira 360°).
- Video grabado a 15 fps con cámara a ~33 Hz: el AVI se reproduce lento.
- En simulación `nodo_video` no graba (tópico distinto) y el bag de sim graba `/bebop1/odom` que no existe sin `odom_viewer`.
- Rutas viejas a `~/ros2_ws/src/control_bebop` (videos, bags de sim y muestra) fuera del repositorio. `bebop1.sdf` tiene ruta absoluta `/home/emiliano/...`.
- En MANUAL, altura y giro van en la cruceta (todo o nada). `control_camara` choca con el joystick si se lanzan juntos.
- `consumo_energ.py` es borrador inservible; `control_vuelo.py` está vacío.
- flake8/pep257 tienen advertencias antiguas.
- Hay `__pycache__` versionados en git (ruido en `git status`).

## 6. Siguiente paso inmediato
Vuelo de prueba escalonado: (1) verificar en el log del driver `MaxAltitude current=2.00 m` y `/bebop/altitude` ≈ 0 en el piso; (2) hover 15 s y comprobar ~1 m; (3) órbita con `yaw_mode: fixed`; (4) solo si sale bien, `lookat`. Después de cada vuelo: `python3 control_bebop/scripts/analisis_bag.py <bag>`. Si la deriva XY sigue siendo el problema, evaluar una referencia externa (cámara cenital, marcadores ArUco en el piso, mocap) o ajustar la estrategia (más hover nativo).

## 7. Comandos útiles
```bash
cd ~/ros2_ws && source /opt/ros/humble/setup.bash
colcon build --packages-select ros2_bebop_driver control_bebop && source install/setup.bash
ros2 launch ros2_bebop_driver bebop_node_launch.xml ip:=192.168.42.1 max_altitude:=2.0
ros2 launch control_bebop real_flight.launch.py          # graba bag automáticamente
ros2 launch control_bebop sim_flight_humble.launch.py     # simulación (Humble)
ros2 launch control_bebop preview_trajectory.launch.py    # sin dron ni Gazebo
python3 control_bebop/scripts/analisis_bag.py control_bebop/bags/vuelo_real_<fecha>
python3 control_bebop/scripts/replay_vuelo.py <bag> --speed 2
cd control_bebop && python3 -m pytest test/test_real_safety_and_frames.py -q
```
Cada cambio en Python/YAML requiere recompilar `control_bebop` (los launches leen `install/`).

## 8. Convenciones para trabajar con este usuario
- Responder en español, claro y directo.
- No tocar el dron real ni lanzar motores desde aquí; solo analizar bags, editar código, compilar y probar sin hardware.
- Antes de commits, push o cambios grandes, confirmar. No hacer commit sin que lo pida.
- Analizar siempre el bag con datos reales antes de proponer cambios de ganancias.
