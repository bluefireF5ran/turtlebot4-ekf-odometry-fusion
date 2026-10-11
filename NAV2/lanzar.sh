#!/usr/bin/env bash
# Atajos para el TurtleBot 4 en simulacion.
#   bash lanzar.sh slam           SLAM + Nav2 + RViz (construir el mapa)
#   bash lanzar.sh guardar-mapa   guarda el mapa actual en mapas/almacen
#   bash lanzar.sh navegar        Nav2 + AMCL sobre mapas/almacen.yaml
#   bash lanzar.sh parar          cierra la simulacion entera (gz incluido)
set -euo pipefail

AQUI="$(cd "$(dirname "$0")" && pwd)"
. /etc/os-release
if [ "${VERSION_ID}" = "24.04" ]; then DISTRO=jazzy; else DISTRO=humble; fi
# shellcheck disable=SC1090
# setup.bash de ROS lee variables sin definir: con set -u abortaria.
set +u
source "/opt/ros/${DISTRO}/setup.bash"
set -u

# Jazzy usa Gazebo Harmonic (turtlebot4_gz_bringup); Humble, Ignition
# Fortress (turtlebot4_ignition_bringup). Mismos argumentos en los dos.
if [ "${DISTRO}" = "jazzy" ]; then
  BRINGUP=(turtlebot4_gz_bringup turtlebot4_gz.launch.py)
else
  BRINGUP=(turtlebot4_ignition_bringup turtlebot4_ignition.launch.py)
fi

mkdir -p "${AQUI}/mapas" "${AQUI}/resultados"

case "${1:-}" in
  slam)
    ros2 launch "${BRINGUP[@]}" slam:=true nav2:=true rviz:=true
    ;;
  guardar-mapa)
    # slam_toolbox publica /map como transient_local; sin esto el saver no lo
    # recibe en sus 2 s por defecto ("Failed to spin map subscription").
    ros2 run nav2_map_server map_saver_cli -f "${AQUI}/mapas/almacen" --ros-args \
      -p map_subscribe_transient_local:=true -p save_map_timeout:=10.0 -p use_sim_time:=true
    echo "Mapa guardado en ${AQUI}/mapas/almacen.yaml"
    ;;
  navegar)
    if [ ! -f "${AQUI}/mapas/almacen.yaml" ]; then
      echo "No hay mapa: primero 'bash lanzar.sh slam' y 'bash lanzar.sh guardar-mapa'."
      exit 1
    fi
    # Sin pose inicial AMCL no publica map->odom, el planner_server agota su
    # espera de 60 s y Nav2 aborta el arranque. El robot aparece en la base,
    # que es el origen del mapa de SLAM: se le da (0, 0, 0) en cuanto AMCL
    # esta activo. Se puede corregir despues con 2D Pose Estimate en RViz.
    (
      until ros2 lifecycle get /amcl 2>/dev/null | grep -q "^active"; do sleep 2; done
      ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
        "{header: {frame_id: map}, pose: {pose: {orientation: {w: 1.0}},
          covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0,
                       0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.07]}}" >/dev/null
      echo ">>> Pose inicial (0, 0, 0) enviada a AMCL"
    ) &
    ros2 launch "${BRINGUP[@]}" localization:=true nav2:=true rviz:=true \
      map:="${AQUI}/mapas/almacen.yaml"
    ;;
  parar)
    # gz sim lanza servidor y GUI en sus propios grupos de procesos: el Ctrl+C
    # al launch no les llega y quedan dos simuladores vivos si se relanza.
    patron="ros2 launch|gz sim|rviz2|parameter_bridge|turtlebot4_|irobot_create_gz|nav2|amcl|_server|lifecycle_manager|slam_toolbox|collision_monitor|velocity_smoother|opennav"
    pids() { ps -eo pid=,cmd= | grep -E "$patron" | grep -vE "grep|lanzar.sh" | awk '{print $1}'; }
    kill -INT $(pids) 2>/dev/null || true; sleep 8
    kill -TERM $(pids) 2>/dev/null || true; sleep 3
    kill -KILL $(pids) 2>/dev/null || true
    echo "Simulacion cerrada."
    ;;
  *)
    echo "Uso: bash lanzar.sh {slam|guardar-mapa|navegar|parar}"
    exit 1
    ;;
esac
