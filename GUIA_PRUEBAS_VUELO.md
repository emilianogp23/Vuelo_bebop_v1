# Guía de pruebas de vuelo con localización visual

Fecha: 19 oct 2026. Paso a paso para dejar listo el corral, calibrar, probar sin volar y hacer los vuelos con la corrección visual (ArUco en vallas + C920 del techo + odometría). El diseño está en [PLAN_LOCALIZACION_ARUCO.md](PLAN_LOCALIZACION_ARUCO.md).

**Regla general:** no se pasa al siguiente paso sin cumplir su criterio. Si algo falla, se vuelve al paso anterior.

| Paso | Qué | ¿Vuela? | Tiempo aprox. |
|---|---|---|---|
| 1 | Software en la PC | no | 30 min |
| 2 | Preparar el corral y los marcadores | no | medio día |
| 3 | Calibrar la C920 | no | 30 min |
| 4 | Prueba con el dron en la mano | no (motores apagados) | 1 h |
| 5 | Vuelo 1: hover en modo sombra | sí | 15 min |
| 6 | Vuelo 2: órbita en modo sombra | sí | 15 min |
| 7 | Vuelo 3: hover con visión al mando | sí | 15 min |
| 8 | Vuelo 4: órbita con visión, `fixed` y luego `lookat` | sí | 30 min |

---

## 1. Software en la PC del laboratorio

### 1.1 Código y compilación
```bash
cd ~/ros2_ws/src
git status                      # los cambios de visión deben estar aquí (o en su rama)
cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash
colcon build --packages-select control_bebop && source install/setup.bash
cd src/control_bebop && python3 -m pytest test/test_vision.py test/test_real_safety_and_frames.py -q
```
Criterio: 25 pruebas pasan.

### 1.2 Parche de altura del driver (obligatorio)
En esta PC el driver **todavía no tiene el parche** (`/bebop/altitude` no existe). Sin él, la z es la velocidad vertical integrada y deriva: la geocerca de techo, el `native_z_guard` y la altura mínima fuera de la zona segura no son confiables (la altura mínima se apaga sola si no hay altura medida).
```bash
cd ~/ros2_ws/src/ros2_bebop_driver
git apply ~/ros2_ws/src/patches/ros2_bebop_driver_altitude.patch
cd ~/ros2_ws && colcon build --packages-select ros2_bebop_driver && source install/setup.bash
```
Si no se puede aplicar, en `config/real_drone.yaml` poner `z_geofence_enabled: false` y `native_z_guard: false`, y volar solo en modo manual hasta resolverlo.

### 1.3 Cámara del techo
```bash
sudo apt install v4l-utils
v4l2-ctl --list-devices          # ver qué /dev/videoN es la C920
v4l2-ctl -d /dev/video0 --list-formats-ext | grep -A3 MJPG   # debe ofrecer 1280x720 a 30 fps
```
- Cable: a 2.74 m de altura hace falta un cable USB activo (con repetidor). Sin él la cámara se desconecta o da cuadros cortados.
- Poner el número de dispositivo en `config/ceiling_camera.yaml` → `device`.
- Si la laptop tiene una webcam integrada, la C920 puede ser `/dev/video2` o similar.

### 1.4 Plot 3D sin dron (para conocerlo)
```bash
ros2 launch control_bebop preview_trajectory.launch.py
```
Abre la vista 3D con el corral, los marcadores, la zona segura (verde), el objeto (prisma café), la órbita objetivo y el dron virtual. Con el ratón se gira la vista; la tecla **R** borra las trayectorias.

| Elemento | Significado |
|---|---|
| Línea magenta | `/bebop1/pose`: lo que usa el control |
| Línea cian | pose corregida por visión (solo en modo sombra) |
| Puntos azules | mediciones de la cámara del Bebop |
| Puntos naranjas | mediciones de la C920 del techo |
| X roja | goal actual |
| Línea roja corta | rumbo del dron |
| Título | estado de vuelo, posición, lock visual; en rojo si está bajo fuera de la zona segura |

---

## 2. Preparar el corral

### 2.1 Origen y ejes
1. Elegir el punto de despegue: es el **origen** del marco corral y el centro de la **zona segura**. Debe estar lejos del objeto y donde no pasen los robots.
2. Marcar en el piso una cruz en el origen y una flecha hacia **+x** (la valla de enfrente al despegar). +y queda a la izquierda.
3. Marcar un círculo de **0.5 m de radio** alrededor del origen (zona segura). Si se quiere otro radio, cambiar `safe_zone_radius`.
4. El dron siempre se coloca en la cruz **mirando hacia +x**.

### 2.2 Objeto
1. Medir el centro del bote respecto al origen: eso va en `target_x`, `target_y` (y `lookat_x`, `lookat_y`) de `real_drone.yaml`.
2. Comprobar que la órbita (radio `circle_radius`, hoy 0.5 m) queda a ≥ 0.6 m de las vallas y no cruza la zona segura más bajo de 1 m.
3. Altura del objeto: 0.95 m (caja 0.70 + bote 0.25). Con 8 marcadores la órbita debe ir a **1.5 m**: en simulación, a 1.5 m la cámara del Bebop ve al menos un marcador en toda la vuelta; a 1.2 m hay ~4 de 36 posiciones sin ninguno (las cubre la C920 o la odometría).

### 2.3 Marcadores
1. Generar los PNG:
   ```bash
   cd ~/ros2_ws/src/control_bebop
   python3 scripts/generar_marcadores.py --salida ~/marcadores
   ```
2. Imprimir **al 100 %** (sin "ajustar a página"), en papel **mate**. Son de 17 cm con 1 cm de margen blanco: caben en hoja carta (una por hoja, orientación vertical, márgenes de impresión mínimos). Medir el cuadro negro con regla: debe dar 17.0 cm (el del dron, 10.0 cm).
3. Pegar cada uno en una placa rígida (foam board o MDF) y fijarla a la valla sin que se mueva ni se doble.
4. Colocación (ids 0–7): **uno centrado en cada tabla** (8 tablas de 2 m × 0.60 m), con el **centro del cuadro negro a 0.30 m** del piso, a 1 m de la esquina:

   | Valla | ids (izq. → der. visto desde dentro) | Hacia dónde mira la cara (`normal_yaw_deg`) |
   |---|---|---|
   | Enfrente (+x) | 0, 1 | 180 |
   | Izquierda (+y) | 2, 3 | −90 |
   | Atrás (−x) | 4, 5 | 0 |
   | Derecha (−y) | 6, 7 | 90 |

   El marcador debe verse "derecho" desde dentro del corral (el número impreso abajo).
5. Marcadores de piso (ids 30–33): **alrededor del objeto**, uno en cada diagonal a ~0.5 m de su centro, borde superior hacia +x (`floor_yaw_deg: 0`). Sirven para calibrar la C920 y, si el objeto está en el piso (Pioneer), son los que ve la cámara del Bebop durante la órbita. Dejarlos fijos; que no los pisen los robots.
6. Marcador del dron (id 40, 10 cm): pegarlo **arriba del Bebop**, centrado, con el **borde superior hacia el frente** del dron, sin tapar hélices ni el sensor. Cartulina ligera.

### 2.3b Objeto en el piso (Pioneer 3DX)
Si se escanea el Pioneer (0.45 × 0.38 m, 0.24 m de alto) en el piso:
- Lanzar con `obj_suelo:=true` (ver §6). La cámara del Bebop se queda en `camera_tilt_deg` (−28°, para ver las vallas) en tierra, despegue y hover, y **baja a `camera_tilt_suelo_deg` (−70°) solo durante la órbita** (estado AUTOMATIC). Al salir de la órbita regresa.
- Con la cámara a −70° no se ve ninguna valla: en la órbita la posición a bordo sale de los **4 marcadores del piso** alrededor del robot (simulado: radio 0.5 m a 1.5 m de altura, siempre ve 2 o más). Sin ellos, la cámara del Bebop no da nada en la órbita y solo queda la C920.
- Tilt según la órbita: `tilt ≈ −atan((h − 0.12) / r)`. Radio 0.5 m y altura 1.5 m → −70°; radio 0.8 m y 1.5 m → −58°. Ajustar `camera_tilt_suelo_deg` en `real_drone.yaml`.
- `target_x`, `target_y` (y `lookat_x`, `lookat_y`) = centro del Pioneer. `yaw_mode: lookat` para que la cámara apunte al robot.
- El Pioneer no debe moverse durante el vuelo. Si se mueve entre vuelos, re-medir su centro y los 4 marcadores del piso.
- La regla de 1 m sigue igual: la órbita va a 1.5 m, muy por encima del robot.

### 2.4 Medir y escribir el mapa
Para cada marcador, medir con cinta la posición del **centro del cuadro negro** respecto al origen (x, y) y su altura z. Precisión ±1 cm: el error de medición se vuelve error de posición del dron.

Editar `control_bebop/config/aruco_map.yaml` (las posiciones actuales son **nominales**, suponen el origen en el centro de un corral de 4×4):
```yaml
  - {id: 0, size: 0.17, center: [1.98, 1.02, 0.30], normal_yaw_deg: 180}
```
Revisar visualmente: `ros2 launch control_bebop preview_trajectory.launch.py` y comprobar en el plot que cada marcador cae en su valla. Después, recompilar (`colcon build --packages-select control_bebop`).

### 2.5 Geocerca
`arena_x_max` (hoy 2.0) es la distancia del origen a la valla **más cercana**: la geocerca es un cuadrado de ±`arena_x_max` alrededor del origen. Si la zona segura no está en el centro, poner la distancia menor, aunque se pierda espacio del lado lejano.

---

## 3. Calibrar la cámara del techo

### 3.1 Intrínsecos (una vez)
1. Imprimir un tablero de ajedrez de 10×7 cuadros (9×6 esquinas internas), cuadro de 25 mm, pegado en una tabla plana. Medir el cuadro real.
2. Correr:
   ```bash
   ros2 run control_bebop calibrar_camara_techo intrinsecos --tablero 9x6 --cuadro 0.025
   ```
3. Mover el tablero por todo el cuadro (centro, esquinas, bordes) y a distintas inclinaciones, a la distancia de vuelo (1.2–1.7 m de la cámara). **ESPACIO** guarda una vista cuando se ven las esquinas de colores; con 20–30 vistas, **C** calibra.

Criterio: error de reproyección < 0.5 px. Si es mayor, repetir con más vistas y el tablero más plano.

### 3.2 Extrínsecos (cada vez que se mueva la cámara)
1. Marcadores de piso 30–33 (y las vallas que se vean) en su lugar; dron y robots fuera de la vista.
2. Correr:
   ```bash
   ros2 run control_bebop calibrar_camara_techo extrinsecos
   ```
Criterio: imprime la posición de la cámara con z ≈ 2.74 m y x, y donde realmente está (±5 cm), con error < 1.5 px.

### 3.3 Cámara del Bebop
Se usan los intrínsecos que publica el driver (`ros2_bebop_driver/config/bebop2_camera_calib.yaml`). Si en el paso 4 la posición a bordo tiene errores > 5 cm de forma sistemática, recalibrarla con `ros-jazzy-camera-calibration` sobre `/bebop/camera/image_raw`.

---

## 4. Prueba con el dron en la mano (motores apagados)

Objetivo: comprobar las dos fuentes y la fusión sin volar.

### 4.1 Lanzar (una terminal por línea)
```bash
ros2 launch ros2_bebop_driver bebop_node_launch.xml ip:=192.168.42.1 max_altitude:=2.0
ros2 run control_bebop real_drone_bridge --ros-args --params-file ~/ros2_ws/src/control_bebop/config/real_drone.yaml
ros2 run control_bebop aruco_localizer
ros2 run control_bebop ceiling_tracker
ros2 run control_bebop plot_3d --ros-args --params-file ~/ros2_ws/src/control_bebop/config/real_drone.yaml
ros2 bag record -o ~/ros2_ws/src/control_bebop/bags/mano_$(date +%Y%m%d_%H%M) /bebop/odom /bebop1/pose /vision/onboard/pose /vision/ceiling/pose /vision/fused_pose /vision/lock
```
No se lanza `real_flight.launch.py` para que no haya joystick ni flight_manager: así no puede despegar.

### 4.2 Qué revisar
1. En los logs: `aruco_localizer` dice "Intrínsecos recibidos" y cada 5 s "Cuadros procesados N, poses M"; `ceiling_tracker` dice "Cuadros N, poses del dron M"; el bridge muestra el offset de la visión.
2. **Puntos de control:** marcar 8–10 puntos en el piso con cinta (incluidos varios sobre la órbita) y sostener el dron encima a 1.2 m (mejor en un tripié o una caja de altura conocida), mirando a +x y luego en otras direcciones. Anotar lo que dan:
   ```bash
   ros2 topic echo /vision/onboard/pose --once --field pose.pose.position
   ros2 topic echo /vision/ceiling/pose --once --field pose.pose.position
   ```
3. **Cobertura del techo:** en los mismos puntos, a 1.0, 1.2 y 1.5 m, anotar si la C920 da poses. Así se sabe qué parte de la órbita cubre.
4. **Tilt:** con el dron a 1.2–1.5 m sobre la órbita, mirando al bote, comprobar que salen poses a bordo. Si no, probar otro tilt: `ros2 run control_bebop aruco_localizer --ros-args -p camera_tilt_deg:=-22.0` (o −35). El valor que funcione va a `camera_tilt_deg` en `real_drone.yaml`.
5. **Recorrido de la órbita:** caminar con el dron la órbita completa a 1.3 m apuntando al bote. En el plot, los puntos azules y naranjas deben seguir el círculo verde y la línea cian debe quedarse sobre él aunque la magenta (odometría) se aleje.

Criterios para volar:
- Cada fuente da error < 5 cm y < 2° en los puntos de control donde ve.
- Las dos fuentes coinciden entre sí en < 5 cm donde ambas ven.
- Durante la órbita a mano, la cámara del Bebop da poses al menos la mitad de los cuadros, o la C920 la cubre.
- El bridge no muestra muchos `rejected`.

---

## 5. Lista de chequeo antes de cada vuelo

- [ ] Batería del dron > 80 % y del mando cargada.
- [ ] Robots del piso detenidos o lejos de la zona segura y de la órbita. Alguien a cargo de ellos.
- [ ] Nadie dentro del corral. El piloto con el mando, otra persona viendo el plot.
- [ ] Dron en la cruz del origen, **mirando a +x**. FlatTrim (Share) en el piso.
- [ ] Log del driver: `MaxAltitude current=2.00 m`; `ros2 topic echo /bebop/altitude` ≈ 0 en el piso.
- [ ] En el plot: el dron aparece en el origen; si la C920 lo ve, título con `LOCK`.
- [ ] Conocer los botones: **L2** deadman (mantener todo el vuelo), **×** despega, **R1** órbita, **□** manual/auto, **○** aterriza, **L1** home, **△** aterrizaje inmediato, **L1+R1+△** corta motores (solo emergencia).

## 6. Vuelo 1: hover en modo sombra

`real_drone.yaml` como viene: `use_vision: true`, `vision_shadow: true`. El control vuela igual que antes; la visión solo se calcula y se graba.

```bash
ros2 launch ros2_bebop_driver bebop_node_launch.xml ip:=192.168.42.1 max_altitude:=2.0
ros2 launch control_bebop real_flight.launch.py
```
El launch abre el plot 3D, el panel de odometría, los dos localizadores y graba el bag en `control_bebop/bags/vuelo_real_<fecha>`.

Para escanear un objeto en el piso (Pioneer 3DX): `ros2 launch control_bebop real_flight.launch.py obj_suelo:=true`. Sin la opción, la cámara se mantiene en −28° todo el vuelo.

1. L2 + × → despega a ~1 m sobre la zona segura.
2. Hover 30–60 s sin tocar nada. Si hace falta subir a 1.2 m, □ (manual) y la cruceta.
3. Observar en el plot: la línea magenta (odometría) se mueve aunque el dron esté quieto; la cian (visión) debe quedarse donde está el dron de verdad.
4. ○ para aterrizar. Medir con cinta dónde quedó el dron.

Criterio: la posición final de la línea cian coincide con la medida (< 10 cm) y la magenta no. Revisar en la salida del bridge que no haya `reinit` frecuentes.

```bash
python3 ~/ros2_ws/src/control_bebop/scripts/analisis_bag.py ~/ros2_ws/src/control_bebop/bags/vuelo_real_<fecha>
```

## 7. Vuelo 2: órbita en modo sombra

Igual que el vuelo 1, con `yaw_mode: fixed`. Después del hover: subir a ~1.5 m en manual, regresar a auto (□) y R1.

Observar: si la órbita vuelve a fallar por deriva, la línea cian lo muestra (el dron real fuera del círculo) aunque la magenta diga que va bien. Es la prueba de que la corrección hace falta y de que la visión la ve.

Criterio: durante toda la órbita la visión tiene lock (título `LOCK` casi siempre) y la cian coincide con lo que se ve a simple vista.

## 8. Vuelo 3: hover con visión al mando

En `config/real_drone.yaml`:
```yaml
    vision_shadow: false
```
Recompilar (`colcon build --packages-select control_bebop`) y lanzar igual.

Qué cambia:
- `/bebop1/pose` ahora es la pose corregida: el control y la geocerca usan la visión.
- R1 se rechaza si no hay lock visual ("Órbita rechazada: sin lock visual").
- Si la visión se pierde más de 2 s en AUTOMATIC, RECOVERY o RETURN_HOME, el dron pasa a HOVER.
- Si la visión se reinicializa (salto de posición), el flight_manager re-fija el goal en la posición actual o pasa a HOVER.

1. Despegar, hover 60 s.
2. Con □ (manual) mover el dron 0.5 m y volver a hover: debe quedarse donde se le dejó, sin irse.
3. L1 (home): debe regresar a la cruz del origen.
4. Aterrizar y medir.

Criterio: en hover el dron se mantiene a < 15 cm de su goal durante 60 s y aterriza a < 15 cm del origen al usar home.

## 9. Vuelo 4: órbita con visión

1. `yaw_mode: fixed`. Despegar, subir a ~1.5 m en manual, auto, R1. Una vuelta completa.
2. Si sale bien, `yaw_mode: lookat` (el dron gira para mirar al bote) y repetir.
3. Revisar el video grabado (`control_bebop/videos/`) para la reconstrucción.

Criterio: órbita completa sin `/trajectory_stalled`, error de seguimiento < 0.15 m (en el plot, la magenta sobre el círculo verde).

Después: con los bags de los vuelos 3 y 4, revisar si `ki_xy` e `int_limit_xy` pueden bajar (se subieron para pelear una deriva que antes no se veía).

---

## 10. Problemas frecuentes

| Síntoma | Causa probable | Qué hacer |
|---|---|---|
| `aruco_localizer`: "Sin camera_info" | driver no corre o sin conexión | revisar Wi-Fi del Bebop y el driver |
| A bordo: cuadros procesados pero 0 poses | tilt equivocado, marcadores lejos, mapa mal | probar otro `camera_tilt_deg`; ver la imagen con `rqt_image_view` (si está instalado) |
| `ceiling_tracker`: "No se pudo abrir la cámara" | número de dispositivo, cable | `v4l2-ctl --list-devices`, cambiar `device` |
| `ceiling_tracker`: "sin extrínsecos" | no se calibró | paso 3.2 |
| Techo: cuadros pero 0 poses | marcador del dron muy chico, desenfocado o fuera de vista | foco fijo, más luz, marcador de 12 cm |
| Bridge: muchos `rejected` | mapa mal medido o extrínsecos viejos | re-medir; repetir 3.2 |
| Las dos fuentes no coinciden | mapa o extrínsecos | comparar contra puntos de control con cinta |
| Bridge: "Visión ... reinit" seguido | mediciones malas aceptadas | revisar mapa; subir `vision_gate_xy` solo si el mapa está bien |
| Plot: título rojo "bajo fuera de zona segura" | dron < 1 m fuera del círculo | la geocerca ya lo sube; revisar `/bebop/altitude` |
| R1 no hace nada | sin lock visual (modo no sombra) | ver título del plot; acercar a marcadores |
| El dron sube solo al salir de la zona segura | altura < `min_z_outside_safe` | normal: subir a 1.2 m antes de la órbita |
| Video del escaneo muy inclinado | tilt del localizador (−28°) | ajustar `camera_tilt_deg`; o `launch_vision:=false` y depender de la C920 |

## 11. Parámetros que se tocan

`control_bebop/config/real_drone.yaml` (recompilar después de cambiarlo):

| Parámetro | Valor | Para qué |
|---|---|---|
| `use_vision` | true | suscribe la fusión a las dos fuentes |
| `vision_shadow` | true → false en el vuelo 3 | sombra: solo compara; false: la visión manda |
| `safe_zone_radius` | 0.5 | radio de la zona segura en el origen (m) |
| `min_z_outside_safe` | 1.0 | altura mínima fuera de la zona segura (m) |
| `target_x`, `target_y`, `lookat_x`, `lookat_y` | medidos | centro del bote respecto al origen |
| `circle_radius` | 0.5 | radio de la órbita |
| `yaw_mode` | fixed → lookat | rumbo durante la órbita |
| `arena_x_max` | ≤ distancia a la valla más cercana | geocerca |
| `vision_gate_xy`, `vision_gate_yaw_deg` | 0.5, 15 | rechazo de mediciones raras |
| `vision_loss_hover_time` | 2.0 | s sin visión antes de HOVER |
| `camera_tilt_deg` | −28 | tilt de la cámara del Bebop (ArUco y video) |
| `camera_tilt_suelo_deg` | −70 | tilt durante la órbita con `obj_suelo:=true` |

Otros archivos: `config/aruco_map.yaml` (mapa medido), `config/ceiling_camera.yaml` (lo escribe la calibración).

Argumentos de `real_flight.launch.py`: `obj_suelo` (objeto en el piso, `false` por defecto), `launch_vision` (cámara del Bebop), `launch_ceiling` (C920), `launch_plot3d` (plot 3D), `launch_gui` (panel de odometría), todos `true` por defecto.
