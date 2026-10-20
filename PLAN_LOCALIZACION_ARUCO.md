# Plan: corrección visual de la posición del Bebop con marcadores ArUco

Fecha: 19 oct 2026. Complementa [CONTEXTO.md](CONTEXTO.md) y [DOCUMENTACION.md](DOCUMENTACION.md).

**Cambio (20 oct):** 8 marcadores en las vallas (uno por tabla de 2 m × 0.60 m, centro a 0.30 m), más 4 en el piso y el de 10 cm del dron. Con uno solo a la vista, la ambigüedad de IPPE se resuelve eligiendo la solución con la cámara nivelada. Lo de abajo sobre 20 marcadores queda como historia.

**Estado (19 oct):** fase 2 programada y probada sin hardware (pruebas unitarias e imagen sintética de punta a punta). Falta: fase 0 (montar y medir marcadores), fase 1 (calibrar), prueba con el dron en la mano, Gazebo (fase 3) y vuelos. Ver §9 y la guía [GUIA_PRUEBAS_VUELO.md](GUIA_PRUEBAS_VUELO.md).

## 1. Problema y objetivo

Hoy la posición XY sale de `/bebop/odom` (flujo óptico integrado), que el `real_drone_bridge` convierte al marco de despegue y publica en `/bebop1/pose` y `/odom_local`. Ese dato acumula error y no ve su propia deriva. En el vuelo del 16 oct el dron derivó 0.5 a 1.5 m hacia atrás mientras la odometría marcaba ~0.25 m, y al aterrizar en (0, 0) la odometría decía (0.57, 0.65). Ningún ajuste del PID corrige una deriva que el sensor no mide.

Objetivo: dar al stack una posición absoluta en el corral (x, y, yaw) con error de pocos centímetros, usando dos fuentes visuales (la cámara frontal del Bebop viendo ArUco en las vallas, y la C920 del techo viendo un ArUco sobre el dron), fusionadas con la odometría para que siga habiendo posición entre detecciones.

Restricciones del espacio:
- Corral de 4 × 4 × 2.5 m.
- Dos robots móviles en el piso: el dron no baja de 1.0 m salvo en una zona segura (despegue y aterrizaje).

## 2. Lo que ya hay y se reutiliza

Revisado en este equipo (Ubuntu 24.04, ROS 2 Jazzy):

| Pieza | Estado |
|---|---|
| OpenCV 4.11 con `cv2.aruco.ArucoDetector` | instalado (pip `opencv-python`) |
| `cv_bridge`, `image_geometry`, `tf_transformations`, `transforms3d`, `scipy` | instalados |
| `robot_localization` | instalado (opción B de fusión) |
| `camera_calibration` (ROS) | **no** instalado; se calibra con un script de OpenCV o se instala `ros-jazzy-camera-calibration` |
| Driver: `/bebop/camera/image_raw` + `camera_info` | ya publica, 856 × 480, calibración en `ros2_bebop_driver/config/bebop2_camera_calib.yaml` (fx ≈ 537, fy ≈ 527, distorsión casi nula) |
| `nodo_video` | ya se suscribe a la imagen; un segundo suscriptor no estorba |
| `control_camara` | mueve el tilt virtual (−85° a +17°) |
| `frames.py` (`TakeoffFrame`, `rotate2d`, …) y pruebas con pytest | base para la lógica nueva sin ROS |
| `safety.py` (`apply_geofence`) | se extiende con altura mínima y zona segura |

Dato útil: el video del Bebop está estabilizado digitalmente (gimbal virtual), así que la cámara no hereda roll ni pitch del cuerpo; solo el yaw y el tilt elegido. Eso simplifica el modelo cámara → cuerpo.

## 3. Arquitectura propuesta

Decisión del usuario (19 oct): usar **las dos fuentes visuales** (marcadores en las paredes vistos por el Bebop y la cámara C920 del techo viendo al dron) y **fusionarlas con la odometría**, que se sigue usando.

```
/bebop/camera/image_raw ─► aruco_localizer (a bordo) ──► /vision/onboard/pose ─┐
C920 del techo (USB) ────► ceiling_tracker ────────────► /vision/ceiling/pose ─┤  PoseWithCovarianceStamped,
                                                                               │  marco "corral"
/bebop/odom ─────────────► real_drone_bridge ◄─────────────────────────────────┘
/bebop/altitude ─────────►   (fusión: odometría + 2 fuentes visuales)
                                   │
                                   ├─► /bebop1/pose   (marco corral si use_vision)
                                   ├─► /odom_local
                                   └─► /vision/status (lock por fuente, edad, nº marcadores, residuo)
```

### 3.1 Marco del mundo
- Marco **corral**: origen en el centro de la **zona segura de despegue**, en el piso; x hacia la valla de enfrente al despegar, y a la izquierda, z arriba. El dron se coloca en el origen mirando a +x, así el marco corral coincide con el de despegue y home, recovery y el centro de la órbita (`target_x/y`) conservan su significado. (Cambio del 19 oct: antes era el centro del corral.)
- Con `use_vision: true, vision_shadow: false` el control usa la pose corregida; con `vision_shadow: true` solo se publica para comparar; con `use_vision: false` queda como antes.

### 3.2 Nodo nuevo `aruco_localizer` (cámara del Bebop, marcadores en las paredes)
1. Se suscribe a `image_raw` + `camera_info` y procesa a 10–15 Hz (salta cuadros, la cámara va a ~30 Hz).
2. Detecta marcadores con `cv2.aruco.ArucoDetector` (refinamiento de esquinas subpíxel).
3. Lee un **mapa de marcadores** (`config/aruco_map.yaml`): id → 4 esquinas en 3D en marco corral.
4. Junta las esquinas de **todos** los marcadores visibles y resuelve una sola `cv2.solvePnP` (SQPNP, luego refinamiento LM). Con un solo marcador usa IPPE y descarta la solución ambigua comparando con la predicción de la odometría.
5. Invierte para obtener la pose de la cámara en el corral y, con el offset fijo cámara → base_link, la pose del dron.
6. Filtros de calidad antes de publicar: error de reproyección < 2 px, marcador de al menos ~20 px de lado, distancia < 4 m, yaw coherente con el anterior.
7. Publica `/vision/onboard/pose` con covarianza que crece con la distancia y baja con el número de marcadores, y con el **timestamp de la imagen**.

### 3.2b Nodo nuevo `ceiling_tracker` (C920 del techo)
1. Lee la C920 directo con `cv2.VideoCapture` (V4L2) en la PC; no hace falta instalar `usb_cam` ni `v4l2_camera`. 1280×720 a 30 fps (a 1080p la C920 baja a ~15–30 fps según la luz).
2. Al arrancar fija **autofoco, exposición y balance de blancos en manual** (`v4l2-ctl` o propiedades de OpenCV): si el foco se mueve, cambian los intrínsecos y la calibración deja de valer.
3. Detecta el ArUco pegado sobre el Bebop (id reservado, p. ej. 40, de ~10 cm) y resuelve su pose con `solvePnP` (IPPE_SQUARE).
4. Con los extrínsecos de la cámara (cámara → corral) obtiene x, y, yaw del dron; z solo para validar contra el sonar.
5. Publica `/vision/ceiling/pose` con el timestamp de captura (mismo reloj de la PC, sin problema de sincronización).

Extrínsecos: 4–6 ArUco en el piso en posiciones medidas del marco corral (en zonas por donde no pasen los robots, o solo durante la calibración). Un `solvePnP` con todos da la pose de la cámara; se guarda en `config/ceiling_camera.yaml` y se repite si alguien mueve la cámara.

La lógica (mapa, PnP, filtros) va en un módulo sin ROS, como `frames.py`, para probarla con pytest usando imágenes sintéticas (`cv2.projectPoints`).

### 3.3 Fusión: odometría + dos fuentes visuales (en el bridge)
La odometría es buena a corto plazo y mala a largo; la visión es al revés y llega con retraso. Se fusionan así:

- El bridge mantiene una transformación 2D **offset** = (dx, dy, dyaw) tal que `pose_corral = offset ∘ pose_odom`. Es el error acumulado de la odometría.
- El offset es el estado de un **filtro de Kalman pequeño** (3 estados): en cada ciclo su incertidumbre crece un poco (la odometría deriva) y cada medición visual la reduce con una ganancia según la covarianza de esa medición. Así las dos cámaras se combinan solas: la que esté más cerca o vea más marcadores pesa más, y si una deja de ver, la otra sigue corrigiendo.
- Guarda un historial de ~2 s de poses de odometría. Cuando llega una medición con timestamp t, interpola la odometría en t, así el retraso de cada cámara no mete error.
- **Gating** por fuente: si la innovación es > 0.5 m o > 15° se descarta; si se descartan varias seguidas y coherentes entre sí, se acepta (re-localización). Si las dos cámaras discrepan más de 0.2 m de forma sostenida, se avisa (mapa o extrínsecos mal medidos).
- z sigue saliendo de `/bebop/altitude` (sonar + barómetro). La z visual solo se usa para avisar si discrepan más de 0.3 m.
- El yaw también se corrige: el magnetómetro en interiores es malo y eso hoy rota mal las velocidades.
- `/odom_local` (velocidad en marco cuerpo, para el término D) no cambia.
- Sin ninguna medición visual el offset se queda fijo y la pose es la odometría de siempre.

Ventajas: unas ~150 líneas de Python sin dependencias nuevas, probables con pytest, y no cambia ningún consumidor (`control_PID`, `flight_manager`, `trajectory_controller` siguen leyendo `/bebop1/pose`).

**Alternativa:** `robot_localization` (`ekf_node`, ya instalado) con la velocidad de `/odom_local` y las dos poses visuales. Es más formal, pero exige TF completas, covarianzas bien puestas en el driver y lidiar con el yaw magnético. Lo dejo como opción si el filtro propio se queda corto.

### 3.4 Seguridad y máquina de estados
- **Lock visual**: `flight_manager` no permite iniciar la órbita (R1) sin lock de al menos una fuente visual en los últimos 0.5 s.
- **Pérdida de visión** (de las dos fuentes): < 2 s sigue con odometría pura; > 2 s pasa a HOVER (hover nativo, igual que la órbita atascada); > 10 s aterrizaje solo si está sobre la zona segura, si no queda en HOVER y espera al piloto.
- **Geocerca en marco corral**: x, y en ±1.6 m (deja 0.4 m a las vallas), z máx 2.0 m.
- **Altura mínima 1.0 m** fuera de la zona segura: nuevo parámetro `min_z` y polígono `safe_zone` en `safety.py`. Si el dron está bajo 1.0 m fuera de la zona, la geocerca bloquea gaz negativo y bloquea el movimiento XY que lo aleje de la zona. Despegue y aterrizaje solo dentro de la zona.

## 4. Diseño de los marcadores

Las vallas miden **60 cm** de alto y el dron vuela a 1.0–1.8 m, así que los marcadores quedan por debajo de la cámara. Cámara: FOV ≈ 77° horizontal y ≈ 49° vertical (de fx, fy).

### Opción recomendada: marcadores en la valla + cámara inclinada hacia abajo
- Diccionario `DICT_4X4_50`, ids 0–19.
- Tamaño: **17 cm** de lado negro (hoja carta) (más 2–3 cm de margen blanco), una sola fila con el centro a ~0.35 m del piso. A 3.6 m (el lado opuesto del corral) ocupa ~37 px.
- Distribución: 5 por valla, cada 0.8 m (−1.6, −0.8, 0, 0.8 y 1.6 m del centro de la valla), 20 en total.
- Tilt de la cámara ≈ **−30°** (`control_camara`). El ángulo hacia una valla va de ~10° (dron a 1.0 m, valla a 3.6 m) a ~55° (dron a 1.8 m, valla a 1 m), y con −30° la cámara cubre de −5.5° a −54.5°: entran casi todos los casos. Se ajusta en la fase 2.
- Ventaja nueva: con el dron en el piso y tilt 0° sí ve los marcadores de la valla de enfrente, así que el **lock se obtiene antes de despegar**.
- Riesgo: los dos robots del piso pueden tapar marcadores. Con 5 por valla y varias vallas a la vista basta con que se vea uno o dos; si los robots tapan mucho, se pasa a la opción B.
- Compromiso con el escaneo: hay un solo stream de video, así que el tilt también afecta la grabación. Si el objeto está en el piso a 0.5 m del dron, ya hace falta mirar hacia abajo para encuadrarlo, y el tilt sirve para las dos cosas. Si el objeto es alto, hay que buscar un tilt que encuadre ambos.

### Opción B: postes sobre la valla
Tubos de PVC o listones fijados a los postes de la valla que suban los marcadores a 1.2–1.8 m (por ejemplo uno por esquina y uno al centro de cada valla, 8 postes). La cámara va casi horizontal, los robots no los tapan y no afecta el tilt del video. Cuesta más montarla y los postes tienen que quedar firmes.

### En ambos casos
- Montaje **rígido y plano** (foam board o MDF). Si la valla es malla o tela, el marcador se mueve y el error de su posición se vuelve error del dron.
- Medir las esquinas de cada marcador con cinta (±1 cm) respecto al origen del corral y escribirlas en `aruco_map.yaml`. Mejor aún: tomar ~30 fotos del corral y refinar el mapa con un ajuste (bundle adjustment) a partir de esas mismas detecciones; se deja como mejora.
- Objeto central: en `lookat` la cámara mira al objeto y tapa parte de la valla de atrás. En la opción A pesa más, porque los marcadores están bajos; se compensa con los de las vallas laterales, que también entran en el FOV de 77°.

### Revisión con el objeto real (bote de 25 cm sobre caja de 70 cm, tope a 0.95 m)
Con órbita de radio 0.5 m, el ángulo hacia abajo al centro del bote y a los marcadores de la valla de enfrente (a ~2.5 m) es:

| Altura del dron | Bote | Marcadores (0.35 m) | Línea de vista sobre la caja |
|---|---|---|---|
| 1.0 m | 19° | 15° | pasa a 0.82 m: la tapa el bote |
| 1.2 m | 37° | 19° | pasa a 0.96 m: justo por encima |
| 1.5 m | 54° | 25° | pasa a 1.18 m: libre |

Con tilt ≈ −28° y el dron a 1.2–1.5 m, el bote y los marcadores entran en el mismo cuadro (±24.5° vertical), así que la opción A es compatible con el escaneo. A 1.0 m la caja tapa la valla de atrás.

## 4b. Cámara del techo: Logitech C920 a 2.74 m

Datos del usuario: una C920, a 2.74 m, ve la mayoría del corral (a nivel del piso).

Cobertura calculada con el FOV de la C920 (~70° × 43°, cámara mirando hacia abajo; **no medido**):

| Altura del dron | Distancia a la cámara | Área que ve a esa altura |
|---|---|---|
| 1.0 m | 1.74 m | 2.45 × 1.38 m |
| 1.2 m | 1.54 m | 2.17 × 1.22 m |
| 1.5 m | 1.24 m | 1.75 × 0.98 m |

A la altura de vuelo el área visible se reduce mucho respecto a la del piso. La órbita de 1 m de diámetro cabe apenas si la cámara está sobre el objeto, y en el lado corto del cuadro puede salirse. Por eso las dos fuentes se complementan: el techo da una posición muy buena cuando el dron está en su cuadro, y las paredes cubren el resto. Si hace falta más cobertura, una segunda cámara o una lente más abierta.

- Marcador sobre el dron: ArUco de 10 cm (cartulina mate ligera, ~10 g) centrado en la cubierta, sin tapar el sensor ni las hélices. A 1.24 m y 720p (fx ≈ 930 px) ocupa ~75 px: sobra resolución.
- El ArUco de arriba también da el yaw del dron, que reemplaza al magnetómetro.
- Cable USB: la C920 a 2.74 m necesita un cable activo o extensión USB con repetidor hasta la PC (USB 2.0 pasivo llega a 5 m). Alternativa: una PC pequeña junto a la cámara que publique por ROS 2 en la red.
- En este equipo no hay `/dev/video*` ni `v4l2-ctl` todavía (`sudo apt install v4l-utils` cuando se conecte).

## 5. Fases

Cada fase tiene un criterio de salida; no se avanza sin cumplirlo.

**Fase 0. Preparar el corral (manual, sin código)**
Imprimir y montar los marcadores (20 en la opción A), fijar el origen del corral, medir posiciones, marcar la zona segura en el piso.
*Salida:* `aruco_map.yaml` con las posiciones medidas.

**Fase 1. Calibrar la cámara del Bebop**
Verificar la calibración del driver con un tablero de ajedrez y un script de OpenCV (o `ros-jazzy-camera-calibration`), con el tilt que se usará en vuelo.
Calibrar también la C920 (intrínsecos con tablero, con el foco fijo) y sus extrínsecos con los ArUco del piso.
*Salida:* error de reproyección < 0.5 px en ambas cámaras y archivos de calibración propios.

**Fase 2. Localizadores sin volar**
Implementar `aruco_localizer`, `ceiling_tracker` y su módulo puro con pruebas pytest (PnP sintético con ruido conocido, mapa, gating). Luego, con el driver conectado y **motores apagados**, el usuario lleva el dron en la mano a 8–10 puntos marcados con cinta a 1.2 m de altura, probando el tilt y contando cuántos marcadores ve en cada punto. En los mismos puntos, a 1.0, 1.2 y 1.5 m, verificar si la C920 ve el marcador del dron (así se mide la cobertura real).
*Salida:* las dos fuentes dan error < 5 cm y < 2° en yaw donde ven, y coinciden entre sí; medir la latencia del video (comparando un giro brusco en odometría y visión).

**Fase 3. Simulación**
Poner los marcadores como texturas en las vallas del mundo de Gazebo (`bebop_gz/worlds/bebop1.sdf`), con la misma geometría, y meter deriva artificial a la odometría simulada. Comparar `/bebop1/pose` fusionada con la pose real de Gazebo durante una órbita completa.
*Salida:* órbita completa con error < 10 cm aun con deriva inyectada. (En este equipo `gz` no está en el PATH; hay que confirmar la instalación de Harmonic o usar la laptop con Fortress.)

**Fase 4. Fusión en el bridge en modo sombra**
Implementar el offset en el bridge con `use_vision` y un modo **sombra**: calcula y publica la pose fusionada en `/vision/fused_pose` pero el control sigue usando la odometría de hoy. Grabar en el bag `/vision/onboard/pose`, `/vision/ceiling/pose`, `/vision/status`, `/vision/fused_pose` y la imagen comprimida a 5 Hz. Extender `analisis_bag.py` para comparar odometría contra visión.
*Salida:* en un vuelo en hover manual, la diferencia odometría-visión crece con el tiempo y la fusionada se mantiene estable y coincide con lo que se mide en el piso al aterrizar.

**Fase 5. Lazo cerrado**
Activar `use_vision: true` (el control usa la pose fusionada), con la geocerca y la altura mínima nuevas. Vuelo escalonado como el de CONTEXTO.md §6: hover 30 s, órbita `yaw_mode: fixed`, luego `lookat`.
*Salida:* órbita completa sin `/trajectory_stalled`, error de seguimiento < 0.15 m, y aterrizaje donde la pose dice.

**Fase 6. Reajuste del PID**
Con una posición que ya no deriva, revisar `ki_xy`/`int_limit_xy` (se subieron para pelear una deriva que no se veía) analizando los bags de la fase 5.

## 6. Archivos que se tocarían (cuando se apruebe)

| Archivo | Cambio |
|---|---|
| `control_bebop/control_bebop/aruco_localizer.py` | nuevo nodo |
| `control_bebop/control_bebop/vision.py` | nuevo, lógica pura (mapa, PnP, gating, offset) |
| `control_bebop/config/aruco_map.yaml` | nuevo, mapa medido (paredes y piso) |
| `control_bebop/control_bebop/ceiling_tracker.py` | nuevo nodo de la C920 |
| `control_bebop/config/ceiling_camera.yaml` | nuevo, intrínsecos y extrínsecos de la C920 |
| `control_bebop/control_bebop/real_drone_bridge.py` | suscripción a las dos poses visuales, Kalman de offset, modo sombra |
| `control_bebop/control_bebop/safety.py` | `min_z` y `safe_zone` |
| `control_bebop/control_bebop/flight_manager.py` | lock visual para R1, HOVER por pérdida de visión |
| `control_bebop/config/real_drone.yaml` | `use_vision`, `vision_shadow`, límites en marco corral |
| `control_bebop/launch/real_flight.launch.py` | lanzar el localizador y grabar los tópicos nuevos |
| `control_bebop/setup.py` | entry point del nodo |
| `control_bebop/test/test_vision.py` | pruebas |
| `bebop_ros/bebop_gz/worlds/bebop1.sdf` | vallas con marcadores (fase 3) |
| `control_bebop/scripts/analisis_bag.py` | comparación odometría-visión |

## 7. Riesgos

- **Latencia del video** (0.1–0.3 s estimado): se cubre con el timestamp de la imagen y el historial de odometría; hay que medirla en la fase 2.
- **Desenfoque por movimiento**: con 0.1 rad/s de órbita es bajo, pero en giros rápidos se perderán detecciones; el gating y la odometría lo cubren.
- **Iluminación del laboratorio**: reflejos sobre marcadores brillantes; imprimir en papel mate.
- **Precisión del mapa**: un error de 2 cm al medir un marcador es 2 cm de error del dron cuando solo se ve ese; por eso varios marcadores visibles a la vez.
- **Wi-Fi**: la imagen ya viaja por el enlace del dron; si se cae, se pierde visión y se cae a odometría y luego a HOVER.
- **Yaw**: corregirlo cambia el marco de las velocidades; probarlo primero en modo sombra.

## 8. Decisiones por defecto tomadas (cambiables)

1. Origen del corral en el centro de la zona segura de despegue, a nivel del piso.
2. Fusión por offset en el bridge, no `robot_localization`.
3. Marcadores de 25 cm, `DICT_4X4_50`, 5 por valla a 0.35 m de altura, cámara con tilt ≈ −30° (opción A). Postes como opción B.
4. HOVER tras 2 s sin visión.
5. Margen de 0.4 m a las vallas en la geocerca.
6. Dos fuentes visuales a la vez (paredes y C920 del techo), fusionadas con la odometría por un Kalman de offset en el bridge.
7. Altura de vuelo del escaneo 1.2–1.5 m.

## 9. Qué quedó implementado (19 oct)

| Archivo | Qué hace |
|---|---|
| `control_bebop/vision.py` | Mapa, detección (OpenCV viejo y nuevo), PnP, pose a bordo y desde el techo, `OdomHistory`, `OffsetFilter` (Kalman de offset) |
| `control_bebop/aruco_localizer.py` | Nodo a bordo → `/vision/onboard/pose`; pone el tilt en −28° |
| `control_bebop/ceiling_tracker.py` | Nodo de la C920 → `/vision/ceiling/pose`; no publica sin extrínsecos |
| `control_bebop/calibrar_camara_techo.py` | `ros2 run control_bebop calibrar_camara_techo intrinsecos|extrinsecos` |
| `scripts/generar_marcadores.py` | PNG imprimibles del mapa (17 cm, caben en hoja carta) |
| `real_drone_bridge.py` | Fusión, modo sombra, `/vision/fused_pose`, `/vision/lock`, `/vision/reset` |
| `safety.py` | Zona segura: fuera del radio y bajo 1.0 m bloquea alejarse y sube |
| `flight_manager.py` | R1 exige lock visual; sin visión > 2 s → HOVER; al reinicializar la visión re-fija goals |
| `config/aruco_map.yaml` | Mapa **nominal** (8 en vallas + 4 en el piso, id 40 en el dron) |
| `config/ceiling_camera.yaml` | C920 con intrínsecos nominales, sin extrínsecos |
| `config/real_drone.yaml` | `use_vision: true`, `vision_shadow: true` (solo compara), zona segura 0.5 m, mínimo 1.0 m |
| `launch/real_flight.launch.py` | Lanza los dos localizadores (`launch_vision`, `launch_ceiling`) y graba los tópicos nuevos |
| `control_bebop/plot_3d.py` | Vista 3D en vivo (corral, marcadores, órbita, odometría, visión, goal); en `real_flight` y `preview_trajectory` |
| `test/test_vision.py` | 12 pruebas; más 2 de zona segura en `test_real_safety_and_frames.py` |

Pendiente y no programado: mundo de Gazebo con marcadores (fase 3), comparación odometría-visión en `analisis_bag.py`, aterrizaje automático tras 10 s sin visión (hoy solo HOVER), geocerca asimétrica si la zona segura no está en el centro del corral (hoy es ±`arena_x_max` alrededor del origen).
