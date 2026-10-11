#!/usr/bin/env python3
"""Recorre una lista de puntos con Nav2 y mide cada objetivo.

Por cada objetivo guarda en resultados/waypoints_<fecha>.csv: vuelta, punto,
resultado (exito / cancelado / fallo), segundos y numero de recuperaciones.

    python3 waypoints.py --puntos "1.0,0.0 1.0,1.0 0.0,1.0 0.0,0.0" --vueltas 3

Las coordenadas son del frame 'map', en metros. Eligelas en RViz con Publish
Point (topic /clicked_point) para que no caigan sobre un obstaculo.
Con 'bash lanzar.sh navegar' (AMCL) hay que dar antes la pose inicial en RViz
con 2D Pose Estimate; con SLAM no hace falta.
"""
import argparse
import csv
import math
import time
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# El TurtleBot 4 aparece en el simulador acoplado a su base de carga. Su
# navegador (subclase de BasicNavigator) sabe desacoplarlo; si no esta, se usa
# el generico y hay que desacoplar a mano:
#   ros2 action send_goal /undock irobot_create_msgs/action/Undock {}
try:
    from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator as Navegador
except ImportError:
    Navegador = BasicNavigator


def pose(nav, x, y, yaw=0.0):
    p = PoseStamped()
    p.header.frame_id = "map"
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.orientation.z = math.sin(yaw / 2)
    p.pose.orientation.w = math.cos(yaw / 2)
    return p


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--puntos", required=True, help='"x1,y1 x2,y2 ..." en metros')
    ap.add_argument("--vueltas", type=int, default=1)
    ap.add_argument("--timeout", type=float, default=120.0, help="segundos por objetivo")
    ap.add_argument("--orientar", action="store_true",
                    help="cada objetivo mira en la direccion de llegada (desde el punto anterior). "
                         "Sin esto miran todos a +x y el robot gira y desgira en cada punto")
    args = ap.parse_args()
    puntos = [tuple(float(v) for v in p.split(",")) for p in args.puntos.split()]

    rclpy.init()
    nav = Navegador()
    # 'robot_localization' hace que no espere a AMCL: asi vale igual con SLAM
    # que con el mapa guardado.
    nav.waitUntilNav2Active(localizer="robot_localization")
    if hasattr(nav, "getDockedStatus") and nav.getDockedStatus():
        print("El robot esta en la base: desacoplando...")
        nav.undock()

    salida = Path(__file__).parent / "resultados"
    salida.mkdir(exist_ok=True)
    fichero = salida / f"waypoints_{datetime.now():%Y%m%d_%H%M%S}.csv"
    filas = []

    for vuelta in range(1, args.vueltas + 1):
        for i, (x, y) in enumerate(puntos, 1):
            inicio = time.monotonic()
            yaw = 0.0
            if args.orientar:
                xa, ya = puntos[i - 2]          # el anterior; para el primero, el ultimo
                yaw = math.atan2(y - ya, x - xa)
            nav.goToPose(pose(nav, x, y, yaw))
            recuperaciones = 0
            cancelado = False
            while not nav.isTaskComplete():
                fb = nav.getFeedback()
                if fb is not None:
                    recuperaciones = fb.number_of_recoveries
                if time.monotonic() - inicio > args.timeout and not cancelado:
                    nav.cancelTask()
                    cancelado = True
            segundos = time.monotonic() - inicio
            r = nav.getResult()
            estado = {TaskResult.SUCCEEDED: "exito",
                      TaskResult.CANCELED: "cancelado",
                      TaskResult.FAILED: "fallo"}.get(r, "desconocido")
            filas.append([vuelta, i, x, y, estado, round(segundos, 1), recuperaciones])
            print(f"vuelta {vuelta} punto {i} ({x}, {y}): {estado} en {segundos:.1f} s, "
                  f"{recuperaciones} recuperaciones")

    with open(fichero, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["vuelta", "punto", "x", "y", "resultado", "segundos", "recuperaciones"])
        w.writerows(filas)

    exitos = sum(1 for f in filas if f[4] == "exito")
    print(f"\n{exitos}/{len(filas)} objetivos alcanzados. CSV: {fichero}")
    # No se llama a nav.lifecycleShutdown(): Nav2 lo lanzaste tu y sigue vivo.
    rclpy.shutdown()


if __name__ == "__main__":
    main()
