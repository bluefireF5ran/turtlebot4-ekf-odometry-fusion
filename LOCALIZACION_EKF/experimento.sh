#!/usr/bin/env bash
# Una ejecucion del experimento: degrada la odometria, la fusiona con la IMU en
# robot_localization, recorre los waypoints con Nav2 y graba todo contra la
# verdad del simulador. Al acabar, evalua.
#
# ANTES, en otra terminal, el simulador con Nav2 y el mapa:
#   bash ../NAV2/lanzar.sh navegar     (y 2D Pose Estimate en RViz)
#
# Uso:
#   bash experimento.sh <nombre> [args para degradar_odom.py]
#   bash experimento.sh base_v102_w105 --escala-v 1.02 --escala-w 1.05 --sesgo-w 0.01
#
# Variables (con valor por defecto):
#   PUNTOS="-3.0,1.0 -3.0,-3.1 3.0,-3.1 3.0,1.0"   waypoints en el frame map
#   VUELTAS=3
#   MUNDO=warehouse                                nombre del mundo de Gazebo
#   IMU_RUIDO_W=0.0015  IMU_SESGO_W=0.0            ruido y sesgo del giroscopo (rad/s)
#
# PUNTOS por defecto: rectangulo de 6 x 4,1 m (~20 m por vuelta) con todos los
# lados a >= 0,79 m de cualquier obstaculo. Rodea la base de carga, en
# (0,16, 0): es mas baja que el LiDAR, Nav2 no la ve y el robot chocaba con
# ella (ver ../NAV2/RESULTADOS.md).
set -euo pipefail

AQUI="$(cd "$(dirname "$0")" && pwd)"
NOMBRE="${1:?Uso: bash experimento.sh <nombre> [args de degradar_odom.py]}"
shift
PUNTOS="${PUNTOS:--3.0,1.0 -3.0,-3.1 3.0,-3.1 3.0,1.0}"
VUELTAS="${VUELTAS:-3}"
MUNDO="${MUNDO:-warehouse}"
IMU_RUIDO_W="${IMU_RUIDO_W:-0.0015}"
IMU_SESGO_W="${IMU_SESGO_W:-0.0}"
SALIDA="${AQUI}/resultados/$(date +%Y%m%d_%H%M%S)_${NOMBRE}"
mkdir -p "${SALIDA}"

. /etc/os-release
if [ "${VERSION_ID}" = "24.04" ]; then DISTRO=jazzy; else DISTRO=humble; fi
# shellcheck disable=SC1090
# setup.bash de ROS lee variables sin definir: con set -u abortaria.
set +u
source "/opt/ros/${DISTRO}/setup.bash"
set -u
SIM=(--ros-args -p use_sim_time:=true)


# Dos trampas al parar los nodos:
#  - En un script sin control de trabajos, lo lanzado con & IGNORA SIGINT, asi
#    que Ctrl+C / kill -INT no les llega. `set -m` lo activa y ademas pone cada
#    uno en su propio grupo de procesos.
#  - `ros2 run` lanza el nodo como proceso hijo: matar solo el PID de ros2 run
#    deja el nodo vivo, y en la siguiente ejecucion habria DOS EKF publicando
#    /odometry/filtered. Por eso se manda la senal al grupo entero (-PID).
# Se usa SIGINT y no SIGTERM para que registrar.py cierre sus ficheros.
set -m
PIDS=()
limpiar() {
  for p in "${PIDS[@]}"; do kill -INT -- "-$p" 2>/dev/null || true; done
  sleep 2
  for p in "${PIDS[@]}"; do kill -KILL -- "-$p" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap limpiar EXIT

echo "odom: $* | imu: --ruido-w ${IMU_RUIDO_W} --sesgo-w ${IMU_SESGO_W} | puntos: ${PUNTOS} x${VUELTAS}" \
  > "${SALIDA}/degradacion.txt"

# IMU. El mundo warehouse no carga el sistema IMU de Gazebo, asi que el sensor
# del Create 3 no publica nada. Se anade en caliente al mundo (entidad 1); solo
# si no esta ya, para no crear dos sistemas IMU.
IMU_GZ="/world/${MUNDO}/model/turtlebot4/link/imu_link/sensor/imu/imu"
if ! gz topic -l | grep -qx "${IMU_GZ}"; then
  gz service -s "/world/${MUNDO}/entity/system/add" --reqtype gz.msgs.EntityPlugin_V \
    --reptype gz.msgs.Boolean --timeout 5000 \
    --req 'entity: {id: 1}, plugins: [{name: "gz::sim::systems::Imu", filename: "gz-sim-imu-system"}]'
  sleep 2
fi
ros2 run ros_gz_bridge parameter_bridge "${IMU_GZ}@sensor_msgs/msg/Imu[gz.msgs.IMU" \
  --ros-args -r "${IMU_GZ}:=/imu_gz" &
PIDS+=($!)
# La de Gazebo es perfecta y con un frame_id fuera del arbol TF: imu_realista.py
# la arregla y le mete ruido de giroscopo MEMS -> /imu.
python3 "${AQUI}/imu_realista.py" --ruido-w "${IMU_RUIDO_W}" --sesgo-w "${IMU_SESGO_W}" "${SIM[@]}" &
PIDS+=($!)

python3 "${AQUI}/degradar_odom.py" "$@" "${SIM[@]}" &
PIDS+=($!)

ros2 run robot_localization ekf_node --ros-args --params-file "${AQUI}/ekf_tb4.yaml" -p use_sim_time:=true &
PIDS+=($!)

python3 "${AQUI}/registrar.py" "${SALIDA}" --slam "${SIM[@]}" &
PIDS+=($!)

sleep 5
if [ "$(ros2 node list | grep -c ekf_filter_node)" -gt 1 ]; then
  echo "Hay mas de un ekf_filter_node vivo (de una ejecucion anterior). Matalos antes:"
  echo "  pkill -INT -f ekf_node"
  exit 1
fi
# --orientar: cada objetivo mira hacia donde llega, asi el rectangulo gira
# siempre al mismo lado (como el cuadrado de UMBmark) y el error de giro se
# acumula. Con orientacion fija el robot giraba y desgiraba en cada esquina y
# los errores de escala de giro se cancelaban.
# En segundo plano + wait para que limpiar() tambien lo pare si se interrumpe.
python3 "${AQUI}/../NAV2/waypoints.py" --puntos "${PUNTOS}" --vueltas "${VUELTAS}" --orientar &
PIDS+=($!)
wait "$!"
cp "$(ls -t "${AQUI}"/../NAV2/resultados/waypoints_*.csv | head -1)" "${SALIDA}/waypoints.csv" || true

limpiar
trap - EXIT
python3 "${AQUI}/evaluar.py" "${SALIDA}"
echo "Resultados en ${SALIDA}"
