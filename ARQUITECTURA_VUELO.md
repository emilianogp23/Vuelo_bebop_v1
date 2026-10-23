# Arquitectura y esquema del vuelo físico

Fecha: 21 oct 2026, actualizado el 22 oct. Resume cómo está armado el sistema, qué cambia entre Gazebo y el Bebop real, qué archivos se configuran y el orden de trabajo antes, durante y después del vuelo. El paso a paso de cada prueba, el montaje de los ArUco, la calibración de la cámara y el diseño de la localización están en [GUIA_PRUEBAS_Y_LOCALIZACION.md](GUIA_PRUEBAS_Y_LOCALIZACION.md).

---
## 1. Arquitectura

Los mismos nodos de control corren en simulación y en el dron real. Solo cambia la capa de abajo, la que habla con el dron y produce `/bebop1/pose`.

```
                 PS5 ─ joy_node ─ joystick_interface ─┐ /joy_commands, /deadman_active
                                                      ▼
                                              flight_manager ── /flight_state
                                         (estados, deadman, geocerca, lock)
                                                      │
                                     trajectory_planner (trajectory_controller.py)
                                         órbita alrededor de (target_x, target_y)
                                                      │ /goal, /goal_vel
                                                 control_PID
                                                      │ cmd_vel
      ┌───────────────────────────── REAL ────────────┴──────────── SIMULACIÓN ─────────────────────┐
      │ real_drone_bridge                                │ Gazebo (bebop1.sdf, parrot_bebop_2)       │
      │   /bebop/cmd_vel = inclinación (pitch/roll)      │   /bebop1/cmd_vel = velocidad (m/s)       │
      │   /bebop/odom + /bebop/altitude ─┐               │   pose exacta → /bebop1/pose_gt ─┐        │
      │                                  ▼               │                                  ▼        │
      │                  VisionFusion (vision.py) ◄──────┼──────────────── sim_odometry: le mete    │
      │                  odometría + ArUco                │                 deriva y usa VisionFusion │
      │                                  │               │                                  │        │
      │          aruco_localizer ◄── cámara del Bebop    │  aruco_localizer ◄── cámara simulada      │
      │          (mueve el tilt a −28°)                  │  (fija a −28° en el modelo)               │
      └──────────────────────────────────┼───────────────┴──────────────────────────────────┼────────┘
                                          ▼                                                  ▼
                       /bebop1/pose  (lo que usa el control; corregida si vision_shadow: false)
                       /vision/fused_pose   corregida, solo en modo sombra
                       /vision/odom_pose    odometría sin corregir, solo con la visión al mando
                       /vision/lock         incertidumbre < vision_max_sigma

      Registro: nodo_video → videos/*.avi      ros2 bag record → bags/vuelo_real_* o vuelo_sim_*
      Vista:    plot_3d (en vivo), gui_odometria
```

| Nodo | Qué hace | Real | Sim |
|---|---|---|---|
| `flight_manager` | Máquina de estados, deadman L2, rechaza R1 sin lock | ✔ | ✔ |
| `trajectory_planner` | Genera la órbita y el rumbo (`fixed` o `lookat`) | ✔ | ✔ |
| `control_PID` | Error de posición → comando | ✔ (inclinación) | ✔ (velocidad) |
| `real_drone_bridge` | Driver ↔ ROS, geocerca en todo el vuelo, altura medida, fusión visual | ✔ | — |
| `sim_odometry` | Pose exacta → odometría con deriva + fusión visual; misma geocerca para los comandos | — | ✔ |
| `aruco_localizer` | ArUco en la imagen → pose del dron (`/vision/onboard/pose`) | ✔ | ✔ |
| `nodo_video` | Graba el video para la reconstrucción | ✔ | ✔ |
| `plot_3d`, `gui_odometria` | Vistas en vivo | ✔ | ✔ |
| `calibrar_camara` | Calibra la cámara del Bebop con un tablero ChArUco (sin volar) | ✔ | — |

---

## 2. ¿Lo que se ve en Gazebo es lo que hará el dron real?

**La lógica y los controles sí, la dinámica no del todo.** Desde el 22 oct la simulación carga `real_drone.yaml` y encima `sim.yaml`: la órbita (centro, radio, `circle_speed`, `yaw_mode`, altura de entrada), la máquina de estados, los tiempos, la visión y el mando son los mismos. △ aterriza, perder el mando 3 s aterriza, en HOVER se manda comando cero y los comandos pasan por `/bridge/cmd_vel_raw` y la misma geocerca (`CommandGate`) en los dos. `sim.yaml` solo cambia lo que es físicamente distinto:

| | Gazebo (`sim.yaml`) | Bebop real (`real_drone.yaml`) |
|---|---|---|
| Comando | velocidad exacta (kp 0.70, kd 0.15, ff 1.0) | inclinación; tarda ~0.7 s en responder (kp 0.25, kd 0.60, ff 0.20) |
| Despegue y aterrizaje | los hace el PID | los hace el autopiloto del Bebop |
| HOVER | comando cero: lo sostiene el controlador de velocidad de Gazebo | comando cero: hover nativo con flujo óptico |
| Altura | exacta, lazo PID (`z_mode: pid`) | `/bebop/altitude` (sonar + barómetro), `z_mode: native` |
| Corral | 4 × 4 m, geocerca de estados ±1.65 m | vallas a ~2.45 m, ±1.8 m |
| Odometría | deriva inventada (escala 1.15, sesgo ~1 cm/s) | la del Bebop: deriva real, distinta en cada vuelo |
| Error de seguimiento esperado | ~0.05 m | ~0.10–0.15 m (vuelos de octubre) |
| Cámara | fija a −28°, 640×480, sin ruido | gimbal virtual, la mueve `aruco_localizer`; 856×480 |

La simulación sirve para comprobar la lógica: estados, órbita, fusión y bags. No sirve para afinar ganancias: en Gazebo el comando es velocidad y en el Bebop es inclinación, así que las ganancias no se pasan de uno a otro.

### Altura de la órbita y encuadre del objeto (importante)

La guía vieja recomendaba orbitar a 1.5 m. **Con la cámara a −28° el bote queda fuera del cuadro a esa altura.** Cálculo con la cámara real (campo vertical ±24.5°, cuadro de 3.5° a 52.5° bajo la horizontal), a 0.41 m del eje del bote (radio 0.5 − 0.09 de la cámara) y bote de 0.70 a 0.95 m:

| Altura del dron | Parte de arriba | Centro | Parte de abajo | ¿Se ve completo? |
|---|---|---|---|---|
| 1.0 m | 7° | 23° | 36° | sí, un poco arriba del centro |
| **1.1 m** | 20° | 34° | 44° | **sí, casi centrado** |
| 1.2 m | 31° | 42° | 51° | sí, pegado abajo |
| 1.5 m | 53° | 60° | 66° | **no** |

Como `min_z_outside_safe` es 1.0 m (la órbita sale de la zona segura), lo recomendable es **subir en manual a ~1.05–1.1 m antes de R1**. En la simulación de hoy, a 1.0 m, la cámara vio ArUco en toda la vuelta (hueco máximo de 5.8 s, nunca perdió el lock). Si se quiere volar más alto, hay que bajar `camera_tilt_deg` con la regla `tilt ≈ −atan((h − 0.825) / 0.41)`.

---

## 3. Archivos de configuración: no hace falta un yaml nuevo

| Archivo | Para qué | Cuándo se toca |
|---|---|---|
| `control_bebop/config/real_drone.yaml` | Todos los parámetros del vuelo real: geocerca, PID, órbita, visión, cámara | Antes de cada tanda de vuelos |
| `control_bebop/config/aruco_map.yaml` | Posición **medida** de cada ArUco del laboratorio | Al colocar o mover marcadores |
| `control_bebop/config/real_drone_calibrated.yaml` | Escala de la odometría (la escribe `analisis_deriva`) | Solo con `use_calibration:=true` |
| `control_bebop/config/bebop2_camera_calib.yaml` | Intrínsecos de la cámara (los escribe `calibrar_camara`) | Al calibrar la cámara |
| `control_bebop/config/sim.yaml`, `aruco_map_sim.yaml` | Solo lo propio de Gazebo; el resto lo toma de `real_drone.yaml` | No afectan al dron real |

Después de editar un yaml: `colcon build --packages-select control_bebop`.

Lo que hay que medir y escribir en `real_drone.yaml`:
- `target_x`, `target_y` (y `lookat_x`, `lookat_y`): centro del bote respecto al origen de despegue.
- `arena_x_max`: distancia del origen a la valla más cercana.
- `vision_shadow`: `true` en los primeros vuelos, `false` cuando la visión ya se comprobó.
- `yaw_mode`: `fixed` primero; `lookat` para el escaneo (con `fixed` la cámara no apunta al objeto).
- `camera_tilt_deg`: −28 (ver §2).

---

## 4. Marcadores

Hay dos generadores y no se mezclan:

| Script | Lee | Produce |
|---|---|---|
| `scripts/generar_marcadores.py` | `aruco_map.yaml` (ids y tamaño) | PNG para **imprimir** (laboratorio) |
| `scripts/generar_aruco_gazebo.py` | `aruco_map_sim.yaml` | Texturas y modelo en `bebop1.sdf` (**simulación**) |

Flujo en el laboratorio (detalle en la guía, §3.4–3.6):
1. `cd ~/ros2_ws/src/control_bebop && python3 scripts/generar_marcadores.py --salida ~/marcadores`
2. Imprimir al 100 %, en papel mate. Medir el cuadro negro: debe dar `size` (hoy 18 cm).
3. Pegar en placa rígida. Ids 0–7: uno por tabla, centro a 0.30 m del piso. Ids 30–33: en el piso, alrededor del objeto.
4. Medir con cinta el centro de cada cuadro negro respecto al origen (±1 cm) y escribirlo en `aruco_map.yaml`. Las posiciones que trae hoy son nominales.
5. Revisar en `ros2 launch control_bebop preview_trajectory.launch.py` que cada marcador cae donde está.

El orden importa: el tamaño impreso tiene que coincidir con `size`. Las posiciones se escriben **después** de pegar los marcadores, con lo que se midió.

---

## 5. Cámara del Bebop

- **Tilt:** no se configura a mano. `aruco_localizer` manda `camera_tilt_deg` (−28°) por `/bebop/move_camera` cada segundo. Con `obj_suelo:=true` baja a `camera_tilt_suelo_deg` (−70°) solo durante la órbita. **No lanzar `control_camara` a la vez**: los dos moverían la cámara.
- **Intrínsecos:** los publica el driver. Por defecto son los de `ros2_bebop_driver/config/bebop2_camera_calib.yaml` (otro Bebop); se calibran con `ros2 run control_bebop calibrar_camara` y un tablero ChArUco (guía §4) y el driver los carga con `camera_calibration_file`.
- **Video:** `nodo_video` graba `/bebop/camera/image_raw` en `control_bebop/videos/`. Es el mismo tilt que usa la localización: lo que encuadre bien al bote también sirve para la reconstrucción.

---

## 6. Antes de volar: ¿hace falta un vuelo manual o una prueba sin volar?

Sí, en dos pasos. Ninguno se salta:

1. **Dron encendido en la mano, motores apagados** (guía §6). El dron tiene que estar prendido para que haya cámara y Wi-Fi. No se lanza `real_flight.launch.py`, así no puede despegar.
   - Comprueba: que el localizador ve los ArUco, que la pose que da está a < 5 cm de puntos marcados en el piso, y en qué partes de la órbita no ve ningún marcador.
   - **No comprueba la corrección de la odometría:** en el piso o en la mano la odometría del Bebop casi no deriva como en vuelo.
2. **Vuelos 1 y 2 en modo sombra** (`vision_shadow: true`, guía §8–9). El control vuela solo con la odometría, como antes, y la corrección se calcula aparte. Aquí sí se ve cómo se comporta: en el plot, la línea magenta (odometría) se aleja y la cian (corregida) debe quedarse donde está el dron de verdad. Al aterrizar, medir con cinta: la cian debe quedar a < 10 cm.

Solo después de eso se pone `vision_shadow: false` (vuelos 3 y 4).

Nuevo desde hoy, revisar en el log del bridge cada 5 s:
- `rejected_z`: mediciones descartadas porque su altura no coincide con `/bebop/altitude` (más de `vision_max_dz` = 0.30 m). Si casi todas salen así, la altura medida y la de los ArUco no concuerdan: subir `vision_max_dz` o revisar el parche de altura. Este filtro no se ha probado en el dron real.

---

## 7. Después del vuelo: lo mismo que en simulación

Todo funciona igual con los bags y videos reales; solo falta la pose exacta de Gazebo.

| Herramienta | Comando | Diferencia en el real |
|---|---|---|
| Diagnóstico | `python3 scripts/analisis_bag.py bags/vuelo_real_<fecha>` | Sin `pose_gt`: el error se mide contra `/goal` y contra la cinta |
| Replay 2D | `python3 scripts/replay_vuelo.py bags/vuelo_real_<fecha>` | Igual |
| Replay 3D | `python3 scripts/replay_3d.py bags/vuelo_real_<fecha>` | No hay línea "Real"; el error se mide contra `/goal`. Toma `real_drone.yaml` solo |
| Reconstrucción 3D | `python3 scripts/reconstruccion_3d.py videos/vuelo_<fecha>.avi` | Usar `--camara auto` (la opción `gazebo` es solo para simulación) |

En el replay 3D:
- **Modo sombra:** "Odometría sola" es `/bebop1/pose` y "Corregida con ArUco" es `/vision/fused_pose`.
- **Visión al mando:** "Corregida" es `/bebop1/pose` y "Odometría sola" sale de `/vision/odom_pose` (solo en bags grabados desde hoy).

Todos los scripts de bags necesitan `source /opt/ros/<distro>/setup.bash` antes. La reconstrucción no necesita ROS (ver [RECONSTRUCCION_3D.md](control_bebop/RECONSTRUCCION_3D.md)).

---

## 8. Pendientes y riesgos conocidos

- **ArUco lejanos:** a más de ~2.5 m, un solo marcador falla 0.3–0.5 m, más de lo que el filtro supone. En simulación descentra el rumbo 15–20° del lado de atrás de la órbita. Pendiente: hacer crecer su σ más rápido con la distancia (`measurement_sigma` en `vision.py`).
- **Filtro de altura (`vision_max_dz`)** sin probar en el dron real (§6).
- **Bags:** revisar que no quede un `ros2 bag record` vivo después de cerrar el launch (`pgrep -fa "ros2 bag"`). Uno colgado llenó 62 GB en una noche.
