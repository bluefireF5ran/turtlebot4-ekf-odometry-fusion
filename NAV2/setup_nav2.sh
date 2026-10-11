#!/usr/bin/env bash
# Comprueba Ubuntu y ROS 2 e instala solo lo que falte para Nav2 + SLAM con el
# TurtleBot 4 en simulacion. Ejecutar con: bash setup_nav2.sh
set -euo pipefail

. /etc/os-release
echo "Ubuntu: ${VERSION_ID} (${VERSION_CODENAME})"

case "${VERSION_ID}" in
  24.04) DISTRO=jazzy ;;
  22.04) DISTRO=humble ;;
  *) echo "Ubuntu ${VERSION_ID} no soportado: el TurtleBot 4 va en 22.04 (Humble) o 24.04 (Jazzy)."; exit 1 ;;
esac
echo "ROS 2 esperado: ${DISTRO}"

if [ ! -f "/opt/ros/${DISTRO}/setup.bash" ]; then
  echo
  echo "No encuentro /opt/ros/${DISTRO}. Instala antes ROS 2 ${DISTRO} desktop siguiendo"
  echo "  https://docs.ros.org/en/${DISTRO}/Installation/Ubuntu-Install-Debs.html"
  echo "y vuelve a ejecutar este script."
  exit 1
fi
# shellcheck disable=SC1090
# setup.bash de ROS lee variables sin definir: con set -u abortaria.
set +u
source "/opt/ros/${DISTRO}/setup.bash"
set -u

PAQUETES=(
  "ros-${DISTRO}-navigation2"
  "ros-${DISTRO}-nav2-bringup"
  "ros-${DISTRO}-nav2-simple-commander"
  "ros-${DISTRO}-slam-toolbox"
  "ros-${DISTRO}-robot-localization"
  "ros-${DISTRO}-turtlebot4-simulator"
  "ros-${DISTRO}-turtlebot4-navigation"
)

FALTAN=()
for p in "${PAQUETES[@]}"; do
  if dpkg -s "$p" >/dev/null 2>&1; then
    echo "  ya instalado: $p"
  else
    echo "  falta:        $p"
    FALTAN+=("$p")
  fi
done

if [ ${#FALTAN[@]} -gt 0 ]; then
  echo
  echo "Instalando ${#FALTAN[@]} paquetes..."
  sudo apt update
  sudo apt install -y "${FALTAN[@]}"
fi

grep -q "/opt/ros/${DISTRO}/setup.bash" ~/.bashrc || \
  echo "source /opt/ros/${DISTRO}/setup.bash" >> ~/.bashrc

echo
echo "Listo. Abre una terminal nueva (o: source ~/.bashrc) y lanza:"
echo "  bash lanzar.sh slam"
