# Reconstrucción 3D a partir del video de la órbita

Primer pipeline de fotogrametría (fase 2 del proyecto). Toma el AVI que graba
`nodo_video` (o una carpeta de imágenes), elige los frames más nítidos
repartidos a lo largo de la órbita y corre COLMAP para obtener las poses de la
cámara, una nube de puntos y una malla.

- Código: `control_bebop/reconstruccion.py` (sin ROS) y
  `scripts/reconstruccion_3d.py` (línea de comandos).
- Pruebas: `python3 -m pytest test/test_reconstruccion.py -q`.

## Instalación (una vez, en cada computadora)

```bash
sudo apt install colmap          # Humble/22.04: 3.7, Jazzy/24.04: 3.9
python3 -c "import cv2, numpy"   # ya vienen con cv_bridge
colmap -h | head -2              # dice "with CUDA" o "without CUDA"
```

El paquete de Ubuntu viene **sin CUDA**: da nube dispersa y malla, pero no
nube densa. Para la nube densa hace falta un COLMAP compilado con CUDA (ver
"Nube densa" abajo).

No hace falta ROS ni compilar el paquete: el script se corre directo.

## Uso

```bash
cd ~/ros2_ws/src/Vuelo_bebop_v1/control_bebop
python3 scripts/reconstruccion_3d.py videos/vuelo_20261019_120000.avi
```

El resultado queda en `control_bebop/reconstrucciones/<nombre_del_video>/`
(no se sube a git):

| Archivo | Qué es |
|---|---|
| `frames/` | Frames elegidos (`frame_<índice>.png`, el índice es el del video) |
| `sparse.ply` | Nube de puntos dispersa (con color) |
| `mesh_sparse_poisson.ply` | Malla Poisson sobre la nube dispersa (aproximada) |
| `sparse_normales.ply` | Nube dispersa con normales (entrada de esa malla) |
| `sparse/0/`, `sparse_txt/0/` | Modelo COLMAP: cámaras, poses e imágenes registradas |
| `dense/` | Imágenes sin distorsión (entrada de la nube densa) |
| `dense.ply`, `mesh_poisson.ply` | Nube densa y su malla, solo con CUDA |
| `resumen.json` | Imágenes registradas, puntos, rutas y tiempo |

Para ver los `.ply`: MeshLab o CloudCompare (`sudo apt install meshlab`), o
abrir la carpeta en la GUI de COLMAP (`colmap gui`, File → Import model →
`sparse/0`) para ver también las poses de la cámara.

### Opciones útiles

| Opción | Defecto | Para qué |
|---|---|---|
| `--frames N` | 120 | Frames a usar. 60 a 150 para una órbita. Más = más lento |
| `--skip-start N`, `--skip-end N` | 0 | Frames a ignorar del despegue y aterrizaje |
| `--blur R` | 0.35 | Descarta tramos con nitidez < R·mediana (0 = no filtra) |
| `--ancho PX` | original | Reduce los frames (más rápido) |
| `--camara gazebo` | auto | Valor inicial de los intrínsecos de la cámara de Gazebo |
| `--matcher` | `orbita` | `orbita`: cada frame con sus vecinos, cerrando el lazo. `exhaustive`: todos contra todos (muy lento sin GPU) |
| `--traslape N` | 8 | Vecinos por frame con `--matcher orbita` |
| `--max-features N` | 4096 | Puntos SIFT por frame |
| `--denso on/off` | auto | Nube densa (auto = solo si COLMAP tiene CUDA) |
| `--solo-frames` | | Solo extrae los frames, sin COLMAP |
| `--salida DIR` | | Otra carpeta de salida |

También acepta una carpeta de imágenes en vez del video (por ejemplo frames
exportados de Gazebo, o tomados con otra cámara):

```bash
python3 scripts/reconstruccion_3d.py ~/frames_orbita/ --frames 80
```

## Cómo funciona

1. **Selección de frames.** Se lee todo el video y se mide la nitidez de cada
   frame (varianza del Laplaciano). El video se parte en N tramos con el mismo
   número de frames y de cada tramo se queda el más nítido. Si el mejor de un
   tramo está muy borroso (movimiento brusco), el tramo se descarta. Se reparte
   por número de frame y no por tiempo, porque el AVI se graba con un fps
   nominal que no coincide con el de la cámara.
2. **Puntos SIFT** en cada frame, con una sola cámara para todo el video
   (`single_camera`): COLMAP estima la focal y la distorsión.
3. **Emparejamiento por órbita.** Cada frame se compara con sus 8 siguientes y
   con dos más lejanos, y el último con los primeros (cierre del lazo).
   Con 80 frames son ~800 pares en vez de 3160 de todos contra todos.
4. **Mapper** (SfM incremental): poses de cámara + nube dispersa. Si quedan
   varios modelos sueltos se exporta el más grande.
5. **Malla sin CUDA.** Se estima la normal de cada punto (PCA de sus vecinos,
   orientada hacia las cámaras que lo ven) y `poisson_mesher` de COLMAP, que
   corre en CPU, arma la malla. Es una malla aproximada, útil para ver la forma
   y para el NBV, no para medir detalle. (`delaunay_mesher` sobre la nube
   dispersa se queda colgado en el COLMAP 3.9 de Ubuntu, por eso no se usa.)
6. **Con CUDA**, además `image_undistorter`, `patch_match_stereo`,
   `stereo_fusion` (nube densa) y `poisson_mesher` sobre ella.

Prueba hecha (19 oct 2026, contenedor sin GPU, COLMAP 3.9.1): video sintético
de 240 frames de una órbita de 0.6 m de radio a 1.3 m de altura con cámara a
−28°, alrededor de una caja de 0.7 m con un contenedor encima; uno de cada 7
frames borroso a propósito. Con 80 frames: 80/80 cámaras registradas, 44 270
puntos, órbita cerrada, en 8.5 min. Con 60 frames: 60/60, 39 542 puntos y
malla Poisson, en 6.5 min. La caja y el contenedor salen con sus proporciones
correctas, y ningún frame borroso quedó elegido.

## Recomendaciones para el vuelo

- **Una vuelta completa** (o un poco más) a velocidad constante. Con
  `circle_speed` 0.10 rad/s son ~63 s.
- **Que el objeto llene buena parte de la imagen** y esté siempre en cuadro.
  Con la cámara a −28° y la órbita de 0.5 m de radio a 1.2–1.5 m, el objeto
  (tapa a ~0.95 m) queda abajo del centro de la imagen; conviene revisarlo en
  los frames antes de correr COLMAP (`--solo-frames`).
- **Textura.** Superficies lisas y de un solo color no dan puntos SIFT. Si el
  contenedor es liso, pegarle cinta o papel con dibujos; el piso con textura
  también ayuda a registrar las cámaras.
- **Luz pareja y sin reflejos.** Evitar contraluz de ventanas.
- **Recortar despegue y aterrizaje** con `--skip-start/--skip-end`: el dron
  girando en el lugar no aporta paralaje y confunde al mapper.

## Si falla

| Síntoma | Qué probar |
|---|---|
| "no pudo reconstruir ningún modelo" o pocas imágenes registradas | Más frames (`--frames 150`), más vecinos (`--traslape 15`), o `--matcher exhaustive` si son pocos frames. Revisar que el objeto se vea y tenga textura |
| Varios modelos (`n_models` > 1 en `resumen.json`) | Hubo un tramo sin traslape (borroso o giro brusco). Bajar `--blur` o subir `--frames` |
| La malla dispersa sale muy pobre | Es lo esperado sin nube densa. Usar un COLMAP con CUDA o OpenMVS sobre `dense/` |
| Muy lento | `--ancho 640`, `--frames 60`, `--max-features 2048` |

## Nube densa

La nube densa de COLMAP (`patch_match_stereo`) solo existe con CUDA. Opciones:

- Compilar COLMAP con CUDA en la computadora que tenga GPU NVIDIA
  (https://colmap.github.io/install.html), o usar la imagen Docker oficial
  `colmap/colmap` con `--gpus all`, y pasar `--colmap <ruta>`.
- Sin GPU: OpenMVS (`InterfaceCOLMAP`, `DensifyPointCloud`, `ReconstructMesh`,
  `TextureMesh`) sobre la carpeta `dense/` que deja este script.

## Limitaciones y siguientes pasos

- **Escala arbitraria.** Fotogrametría con una cámara: la nube no está en
  metros. Para escalarla y ponerla en el marco del corral se puede alinear la
  trayectoria de cámaras de COLMAP con la pose del dron del bag
  (`/bebop1/pose`, o la fusión con ArUco cuando exista) con una similitud
  (Umeyama), o medir un objeto de tamaño conocido.
- **Recorte del objeto.** La nube incluye piso, paredes y caja. Para el NBV
  habrá que recortar el volumen del objeto (por ejemplo con la caja
  conocida, tapa a ~0.95 m).
- Los intrínsecos del Bebop real no están calibrados; si se calibra la
  cámara, pasarlos con `--camera-model OPENCV --camera-params fx,fy,cx,cy,k1,k2,p1,p2`.
