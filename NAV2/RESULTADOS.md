# Resultados: Nav2 + SLAM + AMCL con el TurtleBot 4 simulado

Domingo 4-oct-2026, en el Linux de arranque dual (Ubuntu 24.04, ROS 2 Jazzy,
Gazebo Harmonic, RTX 2070). Todo en simulación.

## Qué se ha montado

1. **SLAM + Nav2** (`bash lanzar.sh slam`): slam_toolbox construyendo el mapa
   del almacén mientras el robot navegaba con objetivos de Nav2 Goal en RViz.
2. **Mapa guardado**: `mapas/almacen.yaml` + `.pgm`, 601 × 1003 celdas a 5 cm
   (unos 30 × 50 m), contorno cerrado.
3. **Nav2 + AMCL** sobre el mapa guardado (`bash lanzar.sh navegar`). Con la
   pose inicial en la base, `map -> base_link` daba (−0,002, −0,056) m frente
   a (0,001, 0,000) m de la verdad del simulador (`/sim_ground_truth_pose`).
4. **Recorrido medido** con `waypoints.py`: 4 puntos elegidos con Publish Point
   (rectángulo de ~6 × 3 m junto a la base), 3 vueltas.

## Cifras (`resultados/waypoints_20261004_123846.csv`)

**12/12 objetivos alcanzados, 1 recuperación en total.**

| Tramo | Distancia aprox. | Vuelta 1 | Vuelta 2 | Vuelta 3 |
|---|---|---|---|---|
| → punto 1 | 6 m (v2, v3) | 27,1 s (*) | 45,7 s, 1 recup. | 39,7 s |
| 1 → 2 | 3 m | 15,9 s | 17,1 s | 16,3 s |
| 2 → 3 | 6 m | 23,3 s | 23,6 s | 23,7 s |
| 3 → 4 | 3 m | 17,2 s | 17,4 s | 17,9 s |

(*) En la vuelta 1 el robot salía de (−0,6, 4,8), no del punto 4: no es
comparable.

**Lo que no cuadra:** 4 → 1 y 2 → 3 miden lo mismo, pero 4 → 1 tarda casi el
doble y tuvo la única recuperación. La recta de 4 → 1 (y ≈ 0) pasa por la
base de carga, que está en (0,157, 0).

Primera hipótesis: «el planificador rodea la base». **Era falsa.** Se repitió
el tramo dos veces más grabando `/plan`, la verdad del simulador y
`/hazard_detection` (`/tmp/tramo41.py`, `/tmp/tramo14.py`):

| Repetición | Tiempo | Recuperaciones | Observado |
|---|---|---|---|
| 4 → 1 | 94,8 s | 6 | primer plan a 0,06 m de la base (trayectoria no grabada: fallo de QoS) |
| 1 → 4 | 47,3 s | 3 | los 44 planes a ≤ 0,04 m de la base; choque (`bump_front_right`) a los 15,6 s en (0,04, 0,17); 8,49 m recorridos frente a 5,99 m en recta |

**Causa:** la base es más baja que el plano del LiDAR. No está en el mapa de
SLAM (celdas libres) ni en ningún costmap: la `voxel_layer` del local y la
`obstacle_layer` del global solo usan `scan`. Para Nav2 es un obstáculo
invisible; el planificador traza la ruta a través de ella, el robot choca,
el reflejo del bumper del Create 3 lo frena y Nav2 entra en recuperaciones
hasta que, a base de girar, la rodea.

Arreglos posibles (no probados): una *keepout zone* con los filtros de costmap
de Nav2, pintar la base en el mapa, o añadir la nube de profundidad de la
cámara OAK-D a la `voxel_layer`.

Limitación: un solo recorrido de 4 puntos y 3 vueltas, en un mundo estático.
Es una medida de que funciona y de su repetibilidad, no una evaluación amplia.

## Qué falló y se arregló en los scripts

1. **`set -u` + `source /opt/ros/jazzy/setup.bash`**: el setup de ROS lee
   variables sin definir y el script abortaba nada más empezar. Afectaba a
   `lanzar.sh`, `setup_nav2.sh` y `../LOCALIZACION_EKF/experimento.sh`. Se
   envuelve el `source` en `set +u` / `set -u`.
2. **Finales de línea de Windows (CRLF)** en `waypoints.py` y en
   `experimento.sh`, `registrar.py` y `evaluar.py` de LOCALIZACION_EKF.
   Convertidos a LF.
3. **`map_saver_cli` no recibía el mapa** ("Failed to spin map subscription"):
   slam_toolbox publica `/map` como *transient_local*. Se añaden
   `map_subscribe_transient_local:=true` y `save_map_timeout:=10.0`.
4. **Nav2 abortaba el arranque con AMCL** si no se daba la pose inicial en
   60 s: sin ella no existe `map -> odom` y `planner_server` no se activa.
   `lanzar.sh navegar` publica ahora `/initialpose` (0, 0, 0) en cuanto AMCL
   está activo; se puede corregir después con 2D Pose Estimate.
5. **Gazebo quedaba vivo al cerrar el launch** (servidor y GUI en sus propios
   grupos de procesos) y al relanzar había dos simuladores a la vez. Nuevo
   `bash lanzar.sh parar` que lo cierra todo.

## Hallazgos para LOCALIZACION_EKF

- **El TurtleBot 4 simulado de Jazzy no publica IMU**: el sensor está en el
  URDF del Create 3, pero el mundo `warehouse.sdf` no carga
  `gz-sim-imu-system` y no hay puente a ROS. El experimento odometría + IMU
  necesita un mundo propio con ese plugin y un `parameter_bridge` a `/imu`.
- Hay verdad del simulador ya en ROS: `/sim_ground_truth_pose`
  (`nav_msgs/Odometry`, frame `warehouse`, hijo `turtlebot4`). Con el robot
  en la base coincide con el origen de `map`.
- `/odom` existe, a ~61 Hz, `odom -> base_link`.
