# Guía de pruebas de vuelo y localización con ArUco

Fecha: 22 oct 2026. Reúne la antigua guía de pruebas (`GUIA_PRUEBAS_VUELO.md`) y el plan de localización (`PLAN_LOCALIZACION_ARUCO.md`), más la calibración de la cámara con ChArUco y las pruebas con el dron encendido sin volar. La arquitectura general está en [ARQUITECTURA_VUELO.md](ARQUITECTURA_VUELO.md) y el detalle de cada nodo en [DOCUMENTACION.md](DOCUMENTACION.md).

**Regla general:** no se pasa al siguiente paso sin cumplir su criterio. Si algo falla, se vuelve al paso anterior.

| Paso | Qué | ¿Vuela? | Tiempo aprox. |
|---|---|---|---|
| 2 | Software en la PC | no | 30 min |
| 3 | Preparar el corral y los ArUco | no | medio día |
| 4 | Calibrar la cámara del Bebop (ChArUco) | no (motores apagados) | 30 min |
| 5 | Probar en simulación | no | 20 min |
| 6 | Pruebas con el dron encendido, sin volar | no (motores apagados) | 1 h |
| 7 | Lista de chequeo | — | 5 min |
| 8 | Vuelo 1: hover en modo sombra | sí | 15 min |
| 9 | Vuelo 2: órbita en modo sombra | sí | 15 min |
| 10 | Vuelo 3: hover con visión al mando | sí | 15 min |
| 11 | Vuelo 4: órbita con visión, `fixed` y luego `lookat` | sí | 30 min |

---

## 1. Cómo funciona la localización

### 1.1 Problema
La posición XY del Bebop sale de `/bebop/odom`: velocidad de flujo óptico integrada. Acumula error y no ve su propia deriva. El 16 oct el dron derivó 0.5 a 1.5 m hacia atrás mientras la odometría marcaba ~0.25 m, y al aterrizar en (0, 0) la odometría decía (0.57, 0.65). Ningún ajuste del PID corrige una deriva que el sensor no mide.

La solución: la posición la sigue dando la odometría, y cada vez que la cámara del Bebop ve ArUco de posición conocida (en las vallas y el piso), se corrige su error acumulado.

### 1.2 Marco "corral"
Origen en el centro de la **zona segura de despegue**, en el piso. **x** hacia la valla de enfrente al despegar, **y a la izquierda del dron**, z arriba. El dron se coloca en el origen mirando a +x, así el marco corral coincide con el marco de despegue y `target_x/y`, home y recovery conservan su significado.

### 1.3 Cadena
```
/bebop/camera/image_raw ─► aruco_localizer ──► /vision/onboard/pose ─┐  (marco corral,
                                                                     │   stamp de la imagen)
/bebop/odom, /bebop/altitude ─► real_drone_bridge ◄──────────────────┘
                                  VisionFusion (vision.py)
                                   ├─► /bebop1/pose   (corregida si vision_shadow: false)
                                   ├─► /vision/fused_pose  (corregida, solo en modo sombra)
                                   ├─► /vision/odom_pose   (sin corregir, con la visión al mando)
                                   └─► /vision/lock, /vision/reset
```
En simulación `sim_odometry` ocupa el lugar del bridge: convierte la pose exacta de Gazebo en una odometría con deriva y usa la misma `VisionFusion`.

### 1.4 `aruco_localizer` (la cámara del Bebop)
1. Procesa la imagen a ≤ 15 Hz y detecta los ArUco del mapa (`config/aruco_map.yaml`, `DICT_4X4_50`), con refinamiento subpíxel.
2. Junta las esquinas de todos los marcadores visibles y resuelve un solo PnP (SQPNP + refinamiento). Con un marcador solo, IPPE da dos soluciones; como el video del Bebop está estabilizado (la cámara no tiene roll), se elige la de cámara nivelada.
3. Descarta: marcadores de menos de 18 px, error de reproyección > 2 px, distancia > 4.5 m.
4. Publica la pose del dron con una σ que crece con la distancia² y baja con √(número de marcadores).
5. Mueve el tilt de la cámara a `camera_tilt_deg` (−28°) cada segundo; con `obj_suelo:=true` lo baja a `camera_tilt_suelo_deg` solo en la órbita.

### 1.5 Fusión (`VisionFusion`, `OffsetFilter` en `vision.py`)
- Estima un **offset** (dx, dy, dyaw) tal que `pose_corral = offset ∘ pose_odometría`: el error acumulado de la odometría. Es un Kalman de 3 estados.
- La incertidumbre crece con el tiempo (`vision_q_xy`, la odometría deriva) y cada medición la reduce según su σ.
- **Arranque:** al despegar el offset vale cero con σ = 5 cm y 3° (el dron está en la cruz mirando a +x). Así R1 funciona aunque todavía no se vea ningún marcador.
- **Retraso del video:** guarda 2 s de odometría e interpola en el instante de la imagen.
- **Gate:** descarta mediciones a más de 3σ de lo esperado. Recién corregido el gate es de ~15 cm; tras un hueco se abre solo. Un error real de 0.5 m entra en ~10 s; una racha de PnP espejo de un marcador lejano (2–3 m de error) se queda fuera. No hay re-localización forzada.
- **Filtro de altura:** una medición cuya z difiere más de `vision_max_dz` (0.30 m) de `/bebop/altitude` se descarta (`rejected_z`).
- **Corrección suave:** el offset aplicado sigue al estimado a ≤ 0.10 m/s y 5°/s, para no darle un escalón al PID.
- **Lock** = incertidumbre < `vision_max_sigma` (0.20 m). Sin marcadores pasa de 0.05 a 0.20 m en ~15 s.

### 1.6 Seguridad ligada a la visión
- Con la visión al mando (`vision_shadow: false`): sin lock, R1 se rechaza; tras `vision_loss_hover_time` (2 s) sin lock en AUTOMATIC, RECOVERY o RETURN_HOME → HOVER.
- **Geocerca en todo el vuelo** (real y simulación, cualquier estado y cualquier comando, también el hover): cuadrado de ±`arena_x_max` en el marco corral. Hacia afuera solo deja ir a (distancia a 1.7 m) / 2.5 s, frena si va más rápido y pasando 1.7 m empuja hacia dentro. Con los modelos del Bebop identificados en los bags y el piloto empujando hacia la valla, el peor caso se detiene en ~2.02 m. Usa la pose corregida por los ArUco si hay lock, también en modo sombra (`geofence_use_vision`). No actúa en tierra ni al aterrizar.
- Techo con la altura medida y **altura mínima de 1.0 m fuera de la zona segura** (robots en el piso): bajo esa altura y fuera del círculo de 0.5 m, la geocerca no deja alejarse y sube el dron. La órbita entra a ≥ 1.05 m (`min_entry_altitude`).

---

## 2. Software en la PC

### 2.1 Código y pruebas
```bash
cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash      # humble en la laptop
colcon build --packages-select control_bebop && source install/setup.bash
cd src/control_bebop && python3 -m pytest test/ -q
```
Criterio: 65 pruebas pasan. `test_flake8` y `test_pep257` fallan por advertencias de estilo antiguas; no indican un error.

### 2.2 Parche del driver (obligatorio)
Publica la altura medida en `/bebop/altitude`, fija el techo de firmware (`max_altitude`) y acepta `camera_calibration_file` en el launch. Ya está aplicado en el escritorio (22 oct). En otra máquina:
```bash
cd ~/ros2_ws/src/ros2_bebop_driver
git apply --check ~/ros2_ws/src/patches/ros2_bebop_driver_altitude.patch   # sin salida = aplica
git apply ~/ros2_ws/src/patches/ros2_bebop_driver_altitude.patch
cd ~/ros2_ws && colcon build --packages-select ros2_bebop_driver && source install/setup.bash
grep -c altitude src/ros2_bebop_driver/src/bebop_driver_node.cpp              # > 0 = aplicado
```
Sin el parche, la z es la velocidad vertical integrada y deriva hasta ~1 m: en `config/real_drone.yaml` hay que poner `z_geofence_enabled: false` y `native_z_guard: false`, y la altura mínima fuera de la zona segura se apaga sola.

### 2.3 Plot 3D sin dron
```bash
ros2 launch control_bebop preview_trajectory.launch.py
```
Corral, marcadores (con su id), zona segura (verde), objeto (prisma café), órbita objetivo y un dron virtual. Ratón: girar la vista; tecla **R**: borrar trayectorias.

| Elemento | Significado |
|---|---|
| Línea magenta | `/bebop1/pose`: lo que usa el control |
| Línea cian | pose corregida por visión (solo en modo sombra) |
| Puntos azules | mediciones de la cámara del Bebop |
| X roja | goal actual |
| Línea roja corta | rumbo del dron |
| Título | estado, posición, lock; en rojo si está bajo fuera de la zona segura |

---

## 3. Preparar el corral y los ArUco

### 3.1 Origen y ejes
1. Elegir el punto de despegue: es el **origen** y el centro de la **zona segura**. Lejos del objeto y de donde pasan los robots.
2. Marcar en el piso una cruz en el origen y una flecha hacia **+x**. **+y queda a la izquierda del dron.**
3. Marcar un círculo de **0.5 m de radio** alrededor del origen (`safe_zone_radius`).
4. El dron siempre se coloca en la cruz **mirando hacia +x**.

### 3.2 Objeto y altura de la órbita
1. Medir el centro del objeto respecto al origen: `target_x`, `target_y` (y `lookat_x`, `lookat_y`) en `real_drone.yaml`.
2. La órbita (radio `circle_radius`, 0.5 m) debe quedar a ≥ 0.6 m de las vallas.
3. **Altura para el bote sobre la caja (0.70–0.95 m) con la cámara a −28°:** orbitar a **~1.05–1.1 m**. Cálculo con la cámara real (campo vertical ±24.5°, cuadro de 3.5° a 52.5° bajo la horizontal), a 0.41 m del eje del bote:

   | Altura del dron | Parte de arriba | Centro | Parte de abajo | ¿Se ve completo? |
   |---|---|---|---|---|
   | 1.0 m | 7° | 23° | 36° | sí, un poco arriba del centro |
   | **1.1 m** | 20° | 34° | 44° | **sí, casi centrado** |
   | 1.2 m | 31° | 42° | 51° | sí, pegado abajo |
   | 1.5 m | 53° | 60° | 66° | **no** |

   Como `min_z_outside_safe` es 1.0 m, subir en manual a ~1.05–1.1 m antes de R1. Para volar más alto, bajar `camera_tilt_deg` con `tilt ≈ −atan((h − 0.825) / 0.41)`. En simulación, a 1.0 m la cámara vio ArUco en toda la vuelta (hueco máximo 5.8 s, sin perder el lock).

### 3.3 Objeto en el piso (Pioneer 3DX)
- Lanzar con `obj_suelo:=true`. La cámara se queda en −28° en tierra, despegue y hover, y **baja a `camera_tilt_suelo_deg` (−70°) solo en la órbita** (AUTOMATIC).
- A −70° no se ve ninguna valla: en la órbita la posición sale de los **4 marcadores del piso** alrededor del robot. Sin ellos solo queda la odometría (lock perdido a los ~15 s).
- Tilt según la órbita: `tilt ≈ −atan((h − 0.12) / r)`. Radio 0.5 m y altura 1.5 m → −70°; radio 0.8 m y 1.5 m → −58°.
- El robot no debe moverse durante el vuelo. Si se mueve entre vuelos, re-medir su centro y los 4 marcadores del piso.

### 3.4 Disposición de los marcadores
Ocho en las vallas, uno por tabla (8 tablas de 2.45 m × 0.60 m), con el centro del cuadro negro a ~0.30 m del piso. Los ids van **de izquierda a derecha vistos desde dentro del corral**:

| Valla (vista desde el dron en la cruz) | ids | `normal_yaw_deg` | Coordenadas |
|---|---|---|---|
| Enfrente (+x) | 0, 1 | **180** | x > 0; id 0 con y > 0, id 1 con y < 0 |
| A la derecha del dron (−y) | 2, 3 | **90** | y < 0; id 2 con x > 0, id 3 con x < 0 |
| Atrás (−x) | 4, 5 | **0** | x < 0; id 4 con y < 0, id 5 con y > 0 |
| A la izquierda del dron (+y) | 6, 7 | **−90** | y > 0; id 6 con x < 0, id 7 con x > 0 |

`normal_yaw_deg` es hacia dónde mira la **cara** del marcador, es decir, hacia el centro del corral. Si se pone hacia afuera, las esquinas quedan en espejo y el PnP falla o da la pose reflejada (la prueba `test_wall_markers_face_inside` lo detecta).

Cuatro en el piso (ids 30–33), alrededor del objeto, uno en cada diagonal a ~0.5 m de su centro, con el borde superior hacia +x (`floor_yaw_deg: 0`). Nominal: 30 en (+0.5, +0.5) y 33 en (−0.5, +0.5) respecto al objeto (los dos a la izquierda), 31 y 32 a la derecha.

La simulación (`config/aruco_map_sim.yaml`, mundo `bebop1.sdf`) usa la misma disposición de ids.

### 3.5 Imprimir y montar
```bash
cd ~/ros2_ws/src/control_bebop
python3 scripts/generar_marcadores.py --salida ~/marcadores
```
1. Imprimir **al 100 %** (sin "ajustar a página") en papel **mate**. El cuadro negro sale del tamaño de `size` en `aruco_map.yaml` (hoy 0.18 m) con 1 cm de margen blanco. Medir el cuadro negro con regla: si no coincide con `size`, corregir `size` con lo medido.
2. Pegar cada uno en una placa rígida (foam board o MDF) fija a la valla, sin que se mueva ni se doble. El número impreso queda abajo, el marcador "derecho" visto desde dentro.

### 3.6 Medir y escribir el mapa
Medir con cinta el **centro del cuadro negro** de cada marcador respecto al origen, ±1 cm: x hacia enfrente, **y positiva a la izquierda**, z la altura. Un error de medición se vuelve error de posición del dron. Escribir en `control_bebop/config/aruco_map.yaml`:
```yaml
  - {id: 2, size: 0.18, center: [1.25, -2.51, 0.31], normal_yaw_deg: 90}
```
Comprobar después:
1. `cd ~/ros2_ws/src/control_bebop && python3 -m pytest test/test_vision.py -q`: revisa que cada marcador mire hacia dentro.
2. `ros2 launch control_bebop preview_trajectory.launch.py`: cada marcador debe caer en su valla, en el orden de la tabla 3.4.
3. Recompilar (`colcon build --packages-select control_bebop`). Los nodos leen primero el mapa del código fuente, pero así queda igual en `install/`.

**Estado del mapa (22 oct):** los ids 0–7 se escribieron con la y al revés (positiva a la derecha) y con las caras hacia afuera; ya se corrigieron. Faltan dos medidas que no cuadran, marcadas `RE-MEDIR` en el yaml: el **id 3** quedó a 3 cm del id 2 (por el orden se espera x ≈ −1.2) y el **id 5** tiene x = −2.00 contra −2.52 del id 4 en la misma valla. Los del piso (30–33) siguen con valores nominales.

### 3.7 Geocerca
`arena_x_max` (2.0) es la distancia del origen a la valla **más cercana**: la geocerca es un cuadrado de ±`arena_x_max` alrededor del origen. Si la zona segura no está en el centro, poner la distancia menor.

---

## 4. Calibrar la cámara del Bebop (tablero ChArUco)

El PnP de los ArUco depende de los intrínsecos de la cámara (K y distorsión). Hoy el driver publica los de `ros2_bebop_driver/config/bebop2_camera_calib.yaml`, una calibración genérica de otro Bebop. Calibrar este dron quita un error sistemático de la posición visual.

**Por qué ChArUco:** es un tablero de ajedrez con un ArUco en cada cuadro blanco. Cada esquina se identifica aunque el tablero se vea incompleto o pegado al borde de la imagen, que es donde más se nota la distorsión. El tablero usa `DICT_5X5_100`, distinto del del corral (`DICT_4X4_50`): `aruco_localizer` nunca lo confunde con un marcador del mapa.

### 4.1 Imprimir el tablero
```bash
cd ~/ros2_ws/src/control_bebop
python3 scripts/generar_charuco.py --salida ~/charuco.png
```
- 7 × 5 cuadros de 3.5 cm (24.5 × 17.5 cm, cabe en hoja carta horizontal), marcadores de 2.6 cm.
- Imprimir **al 100 %** en papel mate y pegar en una placa **rígida y plana** (un tablero doblado arruina la calibración).
- **Medir con regla** el lado de varios cuadros y promediar (p. ej. 3.48 cm). Ese valor va en `square_length`.
- Un tablero más grande da mejor resultado: en tabloide, `--cuadro 0.05 --marcador 0.037`, y pasar los mismos valores al nodo. El número de filas debe ser impar (las versiones de OpenCV 4.6 y ≥ 4.7 dibujan distinto los tableros con filas pares).

### 4.2 Calibrar
Dron encendido, **motores apagados**, sobre una mesa o caja, con la PC conectada a su Wi-Fi. No lanzar `real_flight.launch.py` ni `aruco_localizer` (este último también mueve la cámara).
```bash
# Terminal 1
ros2 launch ros2_bebop_driver bebop_node_launch.xml ip:=192.168.42.1
# Terminal 2 (square_length = lo medido)
ros2 run control_bebop calibrar_camara --ros-args -p square_length:=0.0348
```
El nodo pone la cámara en −28° (el tilt de vuelo; `-p camera_tilt_deg:=.nan` para no moverla) y abre una ventana con las esquinas detectadas. Sin entorno gráfico, ver `/calibracion/imagen` con `rqt_image_view`.

Cómo mover el tablero, a 0.3–1.0 m de la cámara:
1. Recorrer **todo el cuadro**: centro, las cuatro esquinas y los bordes.
2. **Cerca y lejos.**
3. **Inclinado** ±30–45° hacia los lados y hacia arriba y abajo. Sin inclinación la distancia focal queda mal determinada; el nodo avisa si la inclinación máxima es < 20°.
4. En cada posición, **quieto ~1 s**: el nodo solo toma la vista si el tablero no se movió (evita desenfoque y el retraso del video) y si es distinta de las anteriores. En la imagen dice "VISTA TOMADA", "quieto..." o "mueve o inclina el tablero"; los círculos azules marcan dónde ya hay vistas.

Con 30 vistas calibra solo (tecla **c** para calibrar antes, con ≥ 10; **r** reinicia; **q** sale). En el log:
```
Calibración: error RMS 0.31 px (bueno < 0.5, aceptable < 1.0) | peor vista 12: 0.62 px | fx 541.2 ...
fx que publica hoy el driver: 537.3
Guardado en .../control_bebop/config/bebop2_camera_calib.yaml
```
Con RMS > 1 px no guarda: revisar `square_length`, que el tablero esté plano y repetir con **r**.

### 4.3 Usar la calibración
```bash
ros2 launch ros2_bebop_driver bebop_node_launch.xml ip:=192.168.42.1 max_altitude:=2.0 \
    camera_calibration_file:=file://$HOME/ros2_ws/src/control_bebop/config/bebop2_camera_calib.yaml

ros2 topic echo /bebop/camera/camera_info --once --field k     # debe mostrar la K nueva
```
`real_flight.launch.py launch_driver:=true` la toma sola si el archivo existe. `aruco_localizer` lee la K una sola vez al arrancar: reiniciarlo después de cambiar de calibración.

### 4.4 Validar
La validación real es la prueba de puntos de control (§6.5): con la calibración nueva, la posición visual debe tener error < 5 cm y < 2° donde ve marcadores. Si empeora respecto a la del driver, volver a la de fábrica (lanzar el driver sin `camera_calibration_file`) y repetir la calibración con más vistas inclinadas.

---

## 5. Probar en simulación

La simulación usa **los mismos parámetros y controles que el vuelo real**: los launches cargan `config/real_drone.yaml` y encima `config/sim.yaml`, que solo cambia lo propio de Gazebo (ganancias para comando de velocidad, altura con PID, cámara fija a −28°, corral de 4 × 4 m). La órbita, `yaw_mode`, los tiempos, la geocerca de estados, la visión y el mando son los del real.

```bash
ros2 launch control_bebop main_flight.launch.py          # escritorio (Jazzy)
ros2 launch control_bebop sim_flight_humble.launch.py    # laptop (Humble)
# Probar el otro modo de visión sin tocar real_drone.yaml:
ros2 launch control_bebop main_flight.launch.py vision_shadow:=false
```
`sim_odometry` convierte la pose exacta en una odometría que deriva como la del Bebop (escala 1.15, sesgo ~1 cm/s, 0.05°/s de yaw) y la corrige con los 12 ArUco del mundo. La pose exacta queda en `/bebop1/pose_gt`. Si se cambia `aruco_map_sim.yaml`: `python3 scripts/generar_aruco_gazebo.py` y recompilar `bebop_gz` y `control_bebop`.

Criterio: la órbita se completa sin "Posición incierta → HOVER" y `analisis_bag.py` da un error de la pose corregida contra `/bebop1/pose_gt` < 0.3 m (20 oct: 0.10 m en promedio, 0.29 m máximo, mediana de cada medición ArUco 6 cm).

---

## 6. Pruebas con el dron encendido, sin volar

Objetivo: comprobar conexión, sensores, mando y localización visual sin que el dron pueda despegar. **No se lanza `real_flight.launch.py`** (no hay flight_manager, así que ningún botón puede despegarlo). Hélices puestas pero nadie toca × con L2.

### 6.1 Revisión física
- [ ] Hélices sin cuarteaduras y bien apretadas; batería > 80 %.
- [ ] Piso con textura bajo la zona de vuelo (el flujo óptico necesita contraste) y buena luz, sin reflejos sobre los marcadores.
- [ ] Lejos de estructuras metálicas grandes (magnetómetro).

### 6.2 Conexión y driver
```bash
ros2 launch ros2_bebop_driver bebop_node_launch.xml ip:=192.168.42.1 max_altitude:=2.0
```
En el log deben aparecer, en este orden: `Connected`, `Setting max altitude to 2.00 m`, `Streaming is started` y `MaxAltitude current=2.00 m (allowed ... )`. Si no aparece la última, el techo no se aplicó: no volar.

### 6.3 Tópicos
```bash
ros2 topic hz /bebop/odom                # ~15 Hz
ros2 topic hz /bebop/camera/image_raw    # ~30 Hz
ros2 topic hz /bebop/altitude            # ~15 Hz (requiere el parche)
ros2 topic echo /bebop/altitude          # ≈ 0 en el piso
```
Criterio: los tres publican. Si `/bebop/altitude` no existe, el driver no tiene el parche (§2.2).

### 6.4 Mando (sin flight_manager)
```bash
ros2 run joy joy_node --ros-args -p autorepeat_rate:=20.0
ros2 run control_bebop joystick_interface --ros-args --params-file ~/ros2_ws/src/control_bebop/config/ps5_dualsense.yaml
ros2 topic echo /joy_commands      # en otra terminal
ros2 topic echo /deadman_active    # en otra terminal
```
| Botón | Código esperado en `/joy_commands` |
|---|---|
| × | 10 (despegar) |
| ○ | 11 (aterrizar) |
| △ | 12 (aterrizaje inmediato) |
| L1 + R1 + △ | 19 (corte de motores) |
| R1 | 13 (órbita) |
| □ | 14 (manual; en MANUAL manda 13) |
| L1 | 17 (home) |
| Share | 18 (FlatTrim) |
| Options | 16 (reset) |

`/deadman_active` debe ser `true` solo con L2 apretado a más de la mitad. Ojo: para el corte L1 + R1 + △, al apretar L1 y R1 primero también salen 17 y 13; el 19 llega con △ y deja el dron en EMERGENCY de todos modos.

### 6.5 Localización visual en mano
```bash
ros2 run control_bebop real_drone_bridge --ros-args --params-file ~/ros2_ws/src/control_bebop/config/real_drone.yaml
ros2 run control_bebop aruco_localizer --ros-args --params-file ~/ros2_ws/src/control_bebop/config/real_drone.yaml
ros2 run control_bebop plot_3d --ros-args --params-file ~/ros2_ws/src/control_bebop/config/real_drone.yaml
ros2 bag record -s sqlite3 -o ~/ros2_ws/src/control_bebop/bags/mano_$(date +%Y%m%d_%H%M) \
    /bebop/odom /bebop/altitude /bebop1/pose /vision/onboard/pose /vision/fused_pose /vision/lock
```
1. **Logs:** `aruco_localizer` dice "Intrínsecos recibidos" y cada 5 s "Cuadros procesados N, poses M"; el bridge muestra cada 5 s el offset, σ y si hay LOCK.
2. **Rumbo:** con el dron en la cruz mirando a +x, el yaw de `/bebop1/pose` ≈ 0. Girarlo 90° a la izquierda: ≈ +90°.
3. **Puntos de control:** marcar 8–10 puntos en el piso (varios sobre la órbita) y sostener el dron encima a ~1.1 m (mejor sobre un tripié o una caja de altura conocida), mirando a +x y luego en otras direcciones. Anotar:
   ```bash
   ros2 topic echo /vision/onboard/pose --once --field pose.pose.position
   ```
4. **Huecos:** en esos puntos, a 1.0, 1.1 y 1.5 m, anotar si salen poses. Así se sabe en qué partes de la órbita manda solo la odometría.
5. **Tilt:** sobre la órbita, mirando al objeto, deben salir poses. Si no, probar `-p camera_tilt_deg:=-22.0` (o −35) en `aruco_localizer`; el que funcione va a `real_drone.yaml`.
6. **Recorrido de la órbita:** caminar la órbita completa despacio apuntando al objeto. Los puntos azules deben seguir el círculo verde. Anotar si la línea magenta (odometría) se mueve: con motores apagados el Bebop puede no estimar velocidad, y entonces la cian se corrige a pasos limitados por el gate. En ese caso la prueba real de la corrección es el vuelo 1.

Criterios para volar:
- Error < 5 cm y < 2° en los puntos de control donde ve marcadores.
- Ningún hueco sin poses de más de ~10 s en la órbita a mano (con 15 s se pierde el lock).
- Pocos `rejected` y `rejected_z` en el log del bridge.

### 6.6 FlatTrim
Con el dron en el piso, en la cruz: `ros2 topic pub --once /bebop/flattrim std_msgs/msg/Empty` (o Share con `real_flight` en IDLE antes de cada vuelo).

---

## 7. Lista de chequeo antes de cada vuelo

- [ ] Batería del dron > 80 % y del mando cargada.
- [ ] Robots del piso detenidos o lejos de la zona segura y de la órbita, con alguien a cargo.
- [ ] Nadie dentro del corral. Piloto con el mando y otra persona viendo el plot.
- [ ] Dron en la cruz del origen, **mirando a +x**. FlatTrim (Share) en el piso.
- [ ] Log del driver: `MaxAltitude current=2.00 m`; `/bebop/altitude` ≈ 0 en el piso.
- [ ] En el plot el dron aparece en el origen; si ve marcadores, los puntos azules caen ahí.
- [ ] Controles (iguales en simulación y en el real):

| Control | Acción |
|---|---|
| **L2** | Deadman: mantener todo el vuelo. Soltarlo > 0.5 s → HOVER |
| L2 + **×** | Despegar |
| L2 + **R1** | Iniciar o reanudar la órbita |
| L2 + **□** | Alternar MANUAL / AUTOMATIC |
| Stick izquierdo | Adelante/atrás e izquierda/derecha (MANUAL) |
| Cruceta | Subir/bajar y girar (MANUAL, todo o nada) |
| **○** | Aterrizar |
| **△** | Aterrizaje inmediato |
| **L1** | Regresar a home (y aterrizar) |
| **L1 + R1 + △** | Corte total de motores: el dron cae. Solo emergencias |
| Share | FlatTrim (solo en tierra) |
| Options | Salir de EMERGENCY |
| Mando sin señal 3 s | Aterriza solo |

---

## 8. Vuelo 1: hover en modo sombra

`real_drone.yaml` con `use_vision: true`, `vision_shadow: true` (desde el 23 oct viene en `false`: ver vuelo 3). El control vuela con la odometría sola; la corrección solo se calcula y se graba.
```bash
ros2 launch ros2_bebop_driver bebop_node_launch.xml ip:=192.168.42.1 max_altitude:=2.0 \
    camera_calibration_file:=file://$HOME/ros2_ws/src/control_bebop/config/bebop2_camera_calib.yaml
ros2 topic hz /bebop/odom        # antes de despegar: debe publicar (~15 Hz); si no, reiniciar el driver
ros2 launch control_bebop real_flight.launch.py           # obj_suelo:=true para el Pioneer
```
El launch abre el plot 3D y el panel de odometría, lanza el localizador y graba el bag en `control_bebop/bags/vuelo_real_<fecha>`.

1. L2 + × → despega a ~1 m sobre la zona segura.
2. Hover 30–60 s sin tocar nada.
3. En el plot: la magenta (odometría) se mueve aunque el dron esté quieto; la cian (corregida) debe quedarse donde está el dron de verdad.
4. ○ para aterrizar. Medir con cinta dónde quedó.

Criterio: la posición final de la cian coincide con la medida (< 10 cm) y la magenta no; pocos `rejected` en el bridge.
```bash
python3 ~/ros2_ws/src/control_bebop/scripts/analisis_bag.py ~/ros2_ws/src/control_bebop/bags/vuelo_real_<fecha>
```

## 9. Vuelo 2: órbita en modo sombra

Hecho el 23 oct con `lookat` (bag `Bags_respaldo/vuelo_real_20261023_154624`): la odometría se desvió ~1 m en 45 s y la órbita real quedó corrida hacia −y; la corregida quedó a 0.10 m (mediana) de la posición real, con lock todo el vuelo. Huecos de 10 y 14 s en la aproximación: la caja tapa los marcadores 0 y 1.

Igual que el vuelo 1, con `yaw_mode: lookat`. Después del hover: subir a ~1.1 m en manual (□ y cruceta), volver a auto (□) y R1.

Si la órbita vuelve a fallar por deriva, la cian lo muestra (el dron real fuera del círculo) aunque la magenta diga que va bien: es la prueba de que la corrección hace falta y de que la visión la ve.

Criterio: hay lock durante toda la órbita (título `LOCK`) y la cian coincide con lo que se ve a simple vista.

## 10. Vuelo 3: hover con visión al mando

`config/real_drone.yaml` ya trae `vision_shadow: false` (23 oct). Lanzar igual que el vuelo 1. Qué cambia:
- `/bebop1/pose` es la pose corregida: el control y la geocerca usan la visión.
- R1 se rechaza sin lock ("Órbita rechazada: posición incierta").
- Sin lock más de 2 s en AUTOMATIC, RECOVERY o RETURN_HOME (unos 17 s sin ningún marcador) → HOVER.
- Las correcciones entran a ≤ 0.10 m/s: tras un hueco el dron se desliza a su lugar, sin saltos.

1. Despegar, hover 60 s.
2. En manual mover el dron 0.5 m y volver a hover: debe quedarse donde se le dejó.
3. L1 (home): debe regresar a la cruz.
4. Aterrizar y medir.

Criterio: en hover se mantiene a < 15 cm de su goal durante 60 s y aterriza a < 15 cm del origen al usar home.

## 11. Vuelo 4: órbita con visión

1. `yaw_mode: lookat` (el que trae `real_drone.yaml`). Despegar, subir a ~1.1 m en manual, auto, R1. Una vuelta completa.
2. Si el lock se pierde en la aproximación (huecos de 10–14 s en el vuelo 2) el dron pasa a HOVER: esperar a que el plot vuelva a decir `LOCK` y R1 para reanudar.

Ojo con `fixed` y la visión al mando: en Gazebo (22 oct), con la cámara siempre hacia +x, en el tramo de la órbita cercano a x = 1.5 m no se ve ningún ArUco durante ~15 s, se pierde el lock y el dron pasa a HOVER. Con `lookat` la vuelta se completa. Recorriendo la órbita en mano (23 oct), con `fixed` hubo huecos de 19 y 24 s: por eso se vuela en `lookat`.
3. Revisar el video (`control_bebop/videos/`) y probar la reconstrucción: `python3 scripts/reconstruccion_3d.py videos/vuelo_<fecha>.avi` (ruta entre comillas si tiene espacios). Al cartón liso le faltan puntos: pegarle textura (periódico, etiquetas).

Criterio: órbita completa sin `/trajectory_stalled`, error de seguimiento < 0.15 m.

Después: con los bags de los vuelos 3 y 4, revisar si `ki_xy` e `int_limit_xy` pueden bajar (se subieron para pelear una deriva que antes no se veía).

---

## 12. Después de cada vuelo

| Herramienta | Comando | Qué mirar |
|---|---|---|
| Diagnóstico | `python3 scripts/analisis_bag.py bags/vuelo_real_<fecha>` | Estados, error en AUTOMATIC, altura, sección "Corrección visual": mediciones, hueco máximo, % sin lock |
| Replay 3D | `python3 scripts/replay_3d.py bags/vuelo_real_<fecha>` | Odometría sola contra corregida y contra `/goal` |
| Replay 2D | `python3 scripts/replay_vuelo.py bags/vuelo_real_<fecha>` | Trayectoria, z y estados |

Todos necesitan `source /opt/ros/<distro>/setup.bash`. Revisar que no quede un `ros2 bag record` vivo después de cerrar el launch (`pgrep -fa "ros2 bag"`): uno colgado llenó 62 GB en una noche.

---

## 13. Problemas frecuentes

| Síntoma | Causa probable | Qué hacer |
|---|---|---|
| `aruco_localizer`: "Sin camera_info" | driver no corre o sin conexión | revisar Wi-Fi del Bebop y el driver |
| Cuadros procesados pero 0 poses | tilt equivocado, marcadores lejos, mapa mal | otro `camera_tilt_deg`; ver la imagen con `rqt_image_view` |
| Bridge: muchos `rejected` | mapa mal medido, cara al revés, o un marcador lejano visto de lado | correr `test_vision.py`; re-medir; comparar contra puntos de control; subir `min_marker_px` |
| Bridge: muchos `rejected_z` | la altura de los ArUco no coincide con `/bebop/altitude` | revisar el parche de altura y la z del mapa; subir `vision_max_dz` |
| σ sube y "sin lock" en la órbita | tramo largo sin marcadores | más marcadores en ese lado u otro tilt |
| Título rojo "bajo fuera de zona segura" | dron < 1 m fuera del círculo | la geocerca ya lo sube; revisar `/bebop/altitude` |
| R1 no hace nada | sin lock (visión al mando) o sin L2 | ver el título del plot |
| El dron sube solo al salir de la zona segura | altura < `min_z_outside_safe` | normal: subir a ~1.1 m antes de la órbita |
| Video del escaneo con el objeto fuera de cuadro | altura y tilt no cuadran | ver §3.2 |
| `calibrar_camara` no toma vistas | el tablero se mueve, pocas esquinas o vistas repetidas | sostenerlo quieto ~1 s; acercarlo; inclinarlo |

---

## 14. Parámetros que se tocan

`control_bebop/config/real_drone.yaml` (lo usan el vuelo real y la simulación; recompilar después):

| Parámetro | Valor | Para qué |
|---|---|---|
| `use_vision` | true | corrige la odometría con `/vision/onboard/pose` |
| `vision_shadow` | true → false en el vuelo 3 | sombra: solo compara; false: la visión manda |
| `safe_zone_radius` | 0.5 | radio de la zona segura (m) |
| `min_z_outside_safe` | 1.0 | altura mínima fuera de la zona segura (m) |
| `target_x`, `target_y`, `lookat_x`, `lookat_y` | medidos | centro del objeto |
| `circle_radius` | 0.5 | radio de la órbita |
| `yaw_mode` | fixed → lookat | rumbo durante la órbita |
| `arena_x_max` | ≤ distancia a la valla más cercana | geocerca |
| `geofence_lookahead`, `geofence_brake_gain` | 2.5, 2.0 | freno anticipado de la geocerca |
| `geofence_use_vision` | true | la geocerca usa la pose corregida si hay lock |
| `vision_gate_sigma` | 3.0 | rechaza mediciones a más de 3σ |
| `vision_max_sigma` | 0.20 | incertidumbre (m) a partir de la cual no hay lock |
| `vision_q_xy` | 0.0025 | crecimiento de la incertidumbre sin marcadores (m²/s) |
| `vision_offset_rate_xy` | 0.10 | velocidad máxima de la corrección (m/s) |
| `vision_max_dz` | 0.30 | diferencia máxima z visual − altura medida (m) |
| `vision_loss_hover_time` | 2.0 | s sin lock antes de HOVER |
| `camera_tilt_deg` | −28 | tilt de la cámara (ArUco y video) |
| `camera_tilt_suelo_deg` | −70 | tilt en la órbita con `obj_suelo:=true` |

Otros archivos: `config/aruco_map.yaml` (mapa medido), `config/bebop2_camera_calib.yaml` (lo escribe `calibrar_camara`), `config/aruco_map_sim.yaml` y `config/sim.yaml` (solo simulación).

Argumentos de `real_flight.launch.py`: `obj_suelo` (false), `launch_vision`, `launch_plot3d`, `launch_gui` (true), `launch_driver` (false), `ip`, `max_altitude`, `use_calibration`.

---

## 15. Riesgos y pendientes

- **ArUco lejanos:** a más de ~2.5 m, un solo marcador falla 0.3–0.5 m, más de lo que su σ supone. En simulación descentra el rumbo 15–20° del lado de atrás de la órbita. Pendiente: hacer crecer su σ más rápido con la distancia (`measurement_sigma` en `vision.py`).
- **Filtro de altura (`vision_max_dz`):** sin probar en el dron real.
- **Latencia del video** (0.1–0.3 s estimado): se cubre con el stamp de la imagen y el historial de odometría; falta medirla (girar brusco y comparar odometría contra visión en el bag).
- **Desenfoque:** en giros rápidos se pierden detecciones; el gate y la odometría lo cubren.
- **Precisión del mapa:** 2 cm de error en un marcador son 2 cm del dron cuando solo se ve ese.
- **Wi-Fi:** si se cae la imagen, se pierde la visión, manda la odometría y luego HOVER.
- **Yaw:** corregirlo cambia el marco de las velocidades; probarlo primero en sombra.
- **Robots del piso:** pueden tapar marcadores bajos.
- **Geocerca:** el freno anticipado se eligió con modelos identificados, no se ha probado en el dron real. En el vuelo 1, empujar en manual despacio hacia una valla y comprobar que se detiene antes de 2 m (log del bridge: `Geocerca: ['bloqueo_x']`).
- **`yaw_mode: fixed` con visión al mando:** hueco sin marcadores de ~15 s en Gazebo (§11).

---

## 16. Historia de decisiones

- **19 oct:** se diseña la corrección visual. Fusión por offset en el bridge (unas 150 líneas, probadas con pytest) en lugar de `robot_localization`, que exigía TF completas y covarianzas del driver. Marcadores en las vallas con la cámara inclinada (opción A); postes sobre la valla quedaron como opción B si los robots tapan mucho.
- **20 oct:** se descarta la cámara del techo (Logitech C920): a la altura de vuelo el marcador del dron quedaba muy chico. Se borraron `ceiling_tracker`, `calibrar_camara_techo` y `ceiling_camera.yaml`. El filtro arranca en el despegue con offset cero, el gate depende de la incertidumbre (la regla "5 rechazos → reiniciar" aceptaba rachas de PnP espejo) y el lock pasa a significar "incertidumbre < 0.20 m". Simulación con ArUco y odometría con deriva: error 0.10 m promedio.
- **21 oct:** filtro de altura `vision_max_dz` y análisis de la altura de la órbita contra el encuadre (§3.2).
- **22 oct:** parche del driver aplicado en el escritorio; mapa real corregido (y al revés y caras hacia afuera); la simulación usa la misma disposición de ids y los mismos parámetros que el real; calibración de la cámara con ChArUco.
