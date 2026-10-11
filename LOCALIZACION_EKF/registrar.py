#!/usr/bin/env python3
"""Graba en formato TUM las trayectorias de todos los estimadores y la verdad.

    python3 registrar.py resultados/ejecucion_01

Ficheros (t x y z qx qy qz qw, t en segundos de simulacion):
  verdad.tum           pose real del modelo en Gazebo
  odom_simulador.tum   /odom (la buena, la que usa Nav2)
  odom_degradada.tum   /odom_degradada (degradar_odom.py)
  ekf.tum              /odometry/filtered (robot_localization)
  slam.tum             (opcional) map -> base_link, si hay SLAM o AMCL

VERDAD DEL SIMULADOR: el TurtleBot 4 de Jazzy ya la publica en ROS como
/sim_ground_truth_pose (nav_msgs/Odometry, frame 'warehouse', hijo
'turtlebot4', ~55 Hz, QoS best effort). Su origen coincide con el del mapa de
SLAM porque el robot aparece en (0, 0) y alli empezo el mapa.
"""
import argparse
import sys
from pathlib import Path

import rclpy
import rclpy.time
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def linea(t, x, y, z, q):
    return f"{t:.6f} {x:.6f} {y:.6f} {z:.6f} {q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f}\n"


class Registrador(Node):
    def __init__(self, carpeta, tema_verdad, con_slam):
        super().__init__("registrar_trayectorias")
        self.carpeta = Path(carpeta)
        self.carpeta.mkdir(parents=True, exist_ok=True)
        self.ficheros = {}
        self.cuenta = {}
        for tema, nombre in [("/odom", "odom_simulador"), ("/odom_degradada", "odom_degradada"),
                             ("/odometry/filtered", "ekf")]:
            self.create_subscription(Odometry, tema, lambda m, n=nombre: self.odom(m, n), 50)
        # La verdad llega best effort: con la QoS por defecto (reliable) no se
        # recibiria nada.
        self.create_subscription(Odometry, tema_verdad, lambda m: self.odom(m, "verdad"),
                                 qos_profile_sensor_data)
        if con_slam:
            from tf2_ros import Buffer, TransformListener
            self.tf = Buffer()
            self.oyente = TransformListener(self.tf, self)
            self.create_timer(0.1, self.slam)
        self.create_timer(5.0, self.informe)

    def escribir(self, nombre, texto):
        if nombre not in self.ficheros:
            self.ficheros[nombre] = open(self.carpeta / f"{nombre}.tum", "w")
        self.ficheros[nombre].write(texto)
        self.cuenta[nombre] = self.cuenta.get(nombre, 0) + 1

    def odom(self, m, nombre):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p = m.pose.pose.position
        self.escribir(nombre, linea(t, p.x, p.y, p.z, m.pose.pose.orientation))

    def slam(self):
        try:
            tr = self.tf.lookup_transform("map", "base_link", rclpy.time.Time())
        except Exception:
            return
        t = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
        v = tr.transform.translation
        self.escribir("slam", linea(t, v.x, v.y, v.z, tr.transform.rotation))

    def informe(self):
        # Volcado a disco cada 5 s: si el proceso muere sin cerrar, se pierde
        # como mucho eso.
        for f in self.ficheros.values():
            f.flush()
        self.get_logger().info("poses grabadas: " + ", ".join(f"{k}={v}" for k, v in sorted(self.cuenta.items())))
        if "verdad" not in self.cuenta:
            self.get_logger().warn("No llega la verdad del simulador: revisa /sim_ground_truth_pose (ver cabecera).")

    def cerrar(self):
        for f in self.ficheros.values():
            f.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("carpeta")
    ap.add_argument("--tema-verdad", default="/sim_ground_truth_pose")
    ap.add_argument("--slam", action="store_true", help="grabar tambien map -> base_link")
    a, resto = ap.parse_known_args()
    rclpy.init(args=resto)
    n = Registrador(a.carpeta, a.tema_verdad, a.slam)
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, Exception):
        # Con Ctrl+C, Humble lanza KeyboardInterrupt y Jazzy
        # ExternalShutdownException. En los dos casos hay que cerrar ficheros.
        pass
    finally:
        n.cerrar()
        print(f"Guardado en {a.carpeta}", file=sys.stderr)


if __name__ == "__main__":
    main()
