# Contexto del proyecto Vuelo_bebop_v1 (para futuros chats)

Última actualización: 22 oct 2026. Resumen de [DOCUMENTACION.md](DOCUMENTACION.md) más el estado actual. Detalle de pruebas y localización: [GUIA_PRUEBAS_Y_LOCALIZACION.md](GUIA_PRUEBAS_Y_LOCALIZACION.md); real contra simulación: [ARQUITECTURA_VUELO.md](ARQUITECTURA_VUELO.md).

## 1. Qué es
Sistema ROS 2 para volar un Parrot Bebop 2 en órbita circular alrededor de un objeto, con la cámara al centro, y grabar video para reconstruirlo en 3D. Fases: (1) vuelo y video, (2) reconstrucción, (3) NBV (Next Best View), (4) segundo vuelo optimizado. Existen la fase 1 y un primer pipeline de la fase 2 con COLMAP (`scripts/reconstruccion_3d.py`, salidas en `control_bebop/reconstrucciones/`). NBV no existe todavía.

Usuario: estudiante/investigador, trabaja en español, en un laboratorio con un corral de vallas de ~5 × 5 m y dos robots en el piso. Dos computadoras:
- Laptop: Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress, `sim_flight_humble.launch.py`.
- Escritorio: Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, `main_flight.launch.py`. El `python3` del PATH es el de `~/rob_env` (OpenCV 4.11); los nodos corren con `/usr/bin/python3` (OpenCV 4.6).

Repositorio git: `~/ros2_ws/src` (rama `main`; remotas `avance-laptop`, `rama-escritorio`, `claude/*`). Git user: emilianogp23. Bags, videos y reconstrucciones no se suben.

## 2. Estructura
- `control_bebop/`: paquete principal (ament_python). Nodos en `control_bebop/`, launches, `config/`, `scripts/`, `test/`, `resource/meshes/` (texturas y modelos de Gazebo).
- `bebop_ros/`: simulación (copia de juliordzcer/bebop_ros). Solo se usan `bebop_gz` (mundo `bebop1.sdf`, modelo, plugins) y `bebop_demo` (`set_pose`).
- `patches/`: parche al driver (altura medida, techo de firmware, calibración de cámara).
- Externos en `~/ros2_ws/src`, fuera del repo: `ros2_bebop_driver` (jeremyfix, alfa; con el parche aplicado en el escritorio) y `ros2_parrot_arsdk`. También hay paquetes ajenos al proyecto: `drone_gazebo`, `my_robot_project`, `VeranoUG2026-Optimizador_Trayectoria`.
- `drone_gazebo` (Matrice 100, ignorado por git): `matrice_flight.launch.py` (23 oct) usa los nodos de control_bebop sin copiarlos; `config/matrice.yaml` escala la órbita (r 20 m, 6 m de altura, alrededor de la estación de bomberos) y la geocerca (±35 m). Si cambian los nombres de tópicos o parámetros de control_bebop, revisar ese launch. RETURN_HOME y RECOVERY rodean el edificio (`no_fly_boxes` de flight_manager, `route_around` en `safety.py`; vacío por defecto, el Bebop no cambia) y tras L1 aterriza exactamente sobre home. La plataforma quedó en el piso (z del mundo −0.472) y el dron centrado en su placa (1.92, −10).

## 3. Arquitectura
joy_node → joystick_interface → flight_manager (máquina de estados, único que publica `/flight_state`) → trajectory_planner (`/goal`, `/goal_vel`) → control_PID (10 Hz).
- Real: control_PID → `/bridge/cmd_vel_raw` → real_drone_bridge (satura, geocerca, watchdog, fusión visual) → `/bebop/cmd_vel`. El bridge convierte `/bebop/odom` + `/bebop/altitude` al marco de despegue y publica `/bebop1/pose`, `/odom_local`.
- Simulación: Gazebo → `/bebop1/pose_gt` → sim_odometry (odometría con deriva + fusión visual) → `/bebop1/pose`; control_PID → `/bridge/cmd_vel_raw` → sim_odometry (misma geocerca y watchdog) → `/bebop1/cmd_vel`.
- **Geocerca en todo el vuelo** (`CommandGate` en `safety.py`, real y sim): de takeoff a land, todos los estados y comandos, también el hover. Cuadrado ±`arena_x_max` (2.0); velocidad hacia afuera limitada a (distancia a 1.7 m)/2.5 s con freno, empuje pasando 1.7 m. Peor caso con los modelos identificados: se detiene en ~2.02 m (valla ~2.45). Usa la pose corregida por visión si hay lock, también en sombra. En Gazebo: empujón manual a 0.5 m/s hacia la pared → se detuvo en x 1.42; la órbita (hasta 1.51) pasa sin frenarse.
- Visión en los dos: aruco_localizer (cámara del dron, `config/aruco_map*.yaml`) → `/vision/onboard/pose` → `VisionFusion` (Kalman del offset de la odometría). `vision_shadow: true` solo compara (`/vision/fused_pose`); `false` hace que el control use la pose corregida.
- Otros: nodo_video (AVI para la reconstrucción), plot_3d, gui_odometria, calibrar_camara (ChArUco), modo muestra (demo_safety_manager + muestra_odometria_gui).

Estados: 0 IDLE, 1 AUTOMATIC, 2 TAKING_OFF, 3 LANDING, 4 EMERGENCY, 5 HOVER, 6 MANUAL, 7 RECOVERY, 8 RETURN_HOME.

**Controles idénticos en simulación y real:** L2 deadman (todo el vuelo; soltar → HOVER), L2+× despega, L2+R1 órbita, L2+□ manual/auto, ○ aterriza, △ aterrizaje inmediato, L1 home, L1+R1+△ corta motores, Share FlatTrim, Options sale de EMERGENCY, mando mudo 3 s → aterriza. En MANUAL altura y giro van en la cruceta (todo o nada).

**Parámetros:** el real usa `config/real_drone.yaml`; la simulación carga ese mismo archivo y encima `config/sim.yaml`, que solo cambia ganancias (Gazebo recibe velocidad, el Bebop inclinación), `z_mode: pid`, cámara fija y corral de 4 × 4 m. Cada cambio en Python/YAML requiere `colcon build --packages-select control_bebop`.

Marco de despegue = marco corral: origen en la cruz de la zona segura, x al frente del dron al despegar, **y a la izquierda**, z arriba. Órbita por defecto: centro (1, 0), radio 0.5, ω 0.10 rad/s, `yaw_mode: lookat` (desde el 23 oct; con `fixed` hay huecos de visión de 19–24 s), altura de entrada.

Dinámica del Bebop real: pitch/roll es inclinación (≈ aceleración), retardo 0.2–0.9 s; comando cero = hover nativo con flujo óptico; ~0.21 rad/s de giro por unidad. Ganancias reales kp 0.25, kd 0.60 (con velocidad medida), ki 0.05, ff 0.20; las de Gazebo no sirven en el real.

## 4. Estado de la localización visual
- Programada y probada en Gazebo (20 oct): error 0.10 m promedio contra la pose real; la odometría sola acumuló ~1 m.
- Validación en mano en el dron real (23 oct, `scripts/punto_control.py`, resultados en `control_bebop/puntos_control.csv`): con dos marcadores, error de 1 a 10 cm y yaw ≤ 3°; con uno solo, 30–50 cm y 8–14°. Con eso, `real_drone.yaml` usa `min_markers: 2` (no publica pose con un solo marcador) y `max_reproj_px: 4.0` (con 2.0 descartaba todas las poses de dos marcadores). En simulación sigue `min_markers: 1`. Hueco sin marcadores en (1, 0) mirando a +x. Órbita recorrida en mano (bag `orbita_mano_20261023_1446`): en `lookat` hay poses en casi toda la vuelta, con huecos de 4 s como máximo; en `fixed`, huecos de 19 y 24 s (se perdería el lock). Ruido de cuadro a cuadro: mediana 4 cm, p95 12 cm. Con los motores apagados el Bebop no publica odometría ni altitud, así que la fusión y el lock solo se pueden probar volando (vuelo 1).
- Mapa del laboratorio (`aruco_map.yaml`): 0–1 enfrente (+x), 2–3 a la derecha del dron (−y), 4–5 atrás, 6–7 a la izquierda (+y), caras hacia el centro; 30–33 en el piso alrededor del objeto. El 22 oct se corrigieron la y (estaba medida al revés) y las caras (hacia afuera); el 23 oct, la x de los ids 3 (−1.22) y 5 (−2.52). Los del piso son nominales.
- Simulación con la misma disposición de ids (`aruco_map_sim.yaml`, regenerado en `bebop1.sdf`).
- Cámara calibrada el 23 oct (`config/bebop2_camera_calib.yaml`, 30 vistas, RMS 0.61 px; fx 530.4 contra 537.3 de fábrica). **Falta validarla** con los puntos de control (guía §6.5). El tablero impreso no es el de `generar_charuco.py`: es 5 × 7 con `DICT_6X6_250`, así que hay que pasar `-p squares_x:=5 -p squares_y:=7 -p dictionary:=DICT_6X6_250`.

## 5. Cambios del 22 oct 2026 (sin commit)
1. Parche del driver aplicado y compilado en el escritorio; el parche ahora incluye `camera_calibration_file` en el launch y el arreglo de FFmpeg.
2. Mapa ArUco real corregido; prueba `test_wall_markers_face_inside` para los dos mapas.
3. Controles estandarizados: la simulación usa `real_drone.yaml`; △ aterriza y el mando mudo aterriza en los dos; HOVER = comando cero en los dos; nodo `trajectory_planner` con el mismo nombre en todos los launches; argumentos de simulación `launch_gui`, `launch_plot3d` (true) y `vision_shadow`.
4. Mando mudo en AUTOMATIC: antes intentaba LANDING por una transición inválida (llegaba solo vía HOVER); ahora usa `_force_landing`.
5. Modo muestra: antes despegaba con solo presionar L2 y mandaba al driver directo, sin geocerca. Ahora L2+× como el real, por el bridge, y la documentación ya no dice que "no vuela" (sí vuela: hover nativo).
6. Calibración de cámara: `calibracion.py`, nodo `calibrar_camara`, `scripts/generar_charuco.py`, `test_calibracion.py`; `real_flight` pasa la calibración al driver si existe.
7. Bags real y sim graban también `/joy_commands` y `/deadman_active`; `replay_vuelo.py` encuentra los venvs en Humble y Jazzy.
8. `GUIA_PRUEBAS_VUELO.md` y `PLAN_LOCALIZACION_ARUCO.md` se juntaron en `GUIA_PRUEBAS_Y_LOCALIZACION.md` (más pruebas sin volar y calibración).
9. Geocerca para todo el vuelo (ver §3); antes solo existía en el real, no actuaba sobre los ceros del watchdog, bloqueaba sin frenar (el peor caso llegaba a 5 m) y en el aterrizaje fuera de la zona segura empujaba hacia arriba. Órbita con altura mínima `min_entry_altitude` 1.05 m.
10. 65 pruebas pasan (`python3 -m pytest test/ -q`; flake8/pep257 fallan por estilo antiguo).

## 6. Historia relevante
- 13–15 oct: identificación de la dinámica con bags; nuevas ganancias.
- 16 oct (bag `vuelo_real_20261016_093834`): no hizo el círculo. Derivó 0.5–1.5 m hacia atrás, la odometría solo vio ~0.25 m y la z integrada marcó 1.78 m volando a ~1 m. Se agregaron el parche de altura, el integral más fuerte y el HOVER por órbita atascada (`/trajectory_stalled`).
- 19–20 oct: localización con ArUco; se descartó la cámara del techo (C920, resolución insuficiente). Reconstrucción con COLMAP.
- 21 oct: filtro de altura de la visión (`vision_max_dz`), análisis de altura de órbita contra encuadre.
- 23 oct 15:46 (bag `Bags_respaldo/vuelo_real_20261023_154624`): primer vuelo real con órbita, caja de cartón en (1, 0), `lookat`, modo sombra. La odometría se desvió ~1 m en 45 s; el control la siguió bien (RMS 0.12 m contra el goal), así que la órbita real quedó corrida hacia −y y terminó a ~0.3 m de la caja; aterrizaje en manual. **La fusión visual funcionó**: contra la trayectoria real (reconstruida con los ArUco del video) la corregida tuvo mediana 0.10 m y la odometría 0.75 m; lock el 100 % del tiempo en el aire; hueco máximo 14.4 s. El yaw de la odometría estuvo 15–19° desviado desde el despegue (la visión lo midió y lo corrigió). La z visual quedó a −7 cm (mediana) de la altitud medida: el sesgo de +17–21 cm de las pruebas en mano era la altura anotada. El parche de altura funciona (`/bebop1/pose` z = `/bebop/altitude`). El bag `vuelo_real_20261023_154513` es un intento previo sin odometría (38 s).

## 7. Problemas abiertos
- Validados en vuelo real (23 oct): parche de altura, fusión visual en modo sombra, `lookat`. Desde entonces `real_drone.yaml` trae `vision_shadow: false` (visión al mando, también en simulación); falta volarlo (vuelos 3 y 4 de la guía).
- Medir los ArUco del piso.
- Altura de la órbita: con la cámara a −28° el bote (0.70–0.95 m) se encuadra a ~1.05–1.1 m, no a 1.5 m.
- Con `yaw_mode: fixed` y la visión al mando, en Gazebo hay ~15 s sin marcadores cerca de x = 1.5 → se pierde el lock → HOVER. Con `lookat` completa.
- La geocerca nueva no se ha probado en el dron real.
- En simulación la odometría con deriva no se re-ancla entre vuelos (el bridge real sí, en cada takeoff en modo sombra): un segundo vuelo en sombra arranca con el error del primero.
- Con un solo marcador la pose falla 0.3–0.5 m a cualquier distancia: el real exige dos (`min_markers`), lo que deja más huecos en la órbita.
- Magnetómetro en interiores degrada el yaw (la visión lo corrige solo con lock).
- `control_camara` y `muestra_odometria_gui` usan la cruceta, igual que el mando en MANUAL; no lanzar con el vuelo.
- `control_panel` simula el deadman siempre presionado: solo simulación.
- `consumo_energ.py` es borrador inservible; `control_vuelo.py` está vacío.
- Revisar que no quede un `ros2 bag record` colgado tras cerrar un launch (`pgrep -fa "ros2 bag"`).

## 8. Siguiente paso
Vuelo 3 de la guía (hover con la visión al mando; `real_drone.yaml` ya lo trae) y luego vuelo 4 (órbita en `lookat`). Antes de despegar, `ros2 topic hz /bebop/odom` (el intento de las 15:45 del 23 oct no tenía odometría). Ponerle textura a la caja para la reconstrucción. Tras cada vuelo: `python3 control_bebop/scripts/analisis_bag.py <bag>`.

## 9. Comandos útiles
```bash
cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash          # humble en la laptop
colcon build --packages-select ros2_bebop_driver control_bebop && source install/setup.bash
ros2 launch ros2_bebop_driver bebop_node_launch.xml ip:=192.168.42.1 max_altitude:=2.0
ros2 launch control_bebop real_flight.launch.py          # graba bag automáticamente
ros2 launch control_bebop main_flight.launch.py           # simulación (Jazzy); sim_flight_humble en Humble
ros2 launch control_bebop preview_trajectory.launch.py    # sin dron ni Gazebo
ros2 run control_bebop calibrar_camara --ros-args -p square_length:=<medido>
python3 control_bebop/scripts/analisis_bag.py control_bebop/bags/vuelo_real_<fecha>
python3 control_bebop/scripts/replay_3d.py <bag>
cd control_bebop && python3 -m pytest test/ -q
```

## 10. Convenciones para trabajar con este usuario
- Responder en español, claro y directo.
- No tocar el dron real ni lanzar motores desde aquí; solo analizar bags, editar código, compilar y probar sin hardware.
- Antes de commits, push o cambios grandes, confirmar. No hacer commit sin que lo pida.
- Analizar siempre el bag con datos reales antes de proponer cambios de ganancias.
