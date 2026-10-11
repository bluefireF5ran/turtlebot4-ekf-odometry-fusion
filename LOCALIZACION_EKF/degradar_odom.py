#!/usr/bin/env python3
"""Republica la odometria del simulador con los errores de un robot real.

En Gazebo la odometria de ruedas es casi perfecta: no derrapa, el radio de la
rueda es exacto y la distancia entre ruedas tambien. Asi no hay nada que
fusionar. Este nodo mete los dos errores sistematicos clasicos de un robot
diferencial (Borenstein y Feng, el test UMBmark):

  --escala-v      error de radio de rueda: la velocidad lineal sale escalada
  --escala-w      error de distancia entre ruedas: el giro sale escalado
  --sesgo-w       sesgo en la velocidad de giro (rad/s)
  --ruido-v / --ruido-w   ruido blanco gaussiano

Publica /odom_degradada (nav_msgs/Odometry) integrando su propia pose a partir
de las velocidades degradadas, con covarianzas coherentes para que el EKF sepa
cuanto fiarse. NO publica TF: Nav2 sigue usando la odometria buena del
simulador y el experimento no le afecta.

    python3 degradar_odom.py --escala-v 1.02 --escala-w 1.05 --sesgo-w 0.01
"""
import argparse
import math
import random

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class Degradador(Node):
    def __init__(self, a):
        super().__init__("degradar_odom")
        self.a = a
        self.x = self.y = self.yaw = 0.0
        self.t_previo = None
        self.pub = self.create_publisher(Odometry, a.salida, 10)
        self.create_subscription(Odometry, a.entrada, self.recibir, 10)
        self.get_logger().info(f"{a.entrada} -> {a.salida}  escala_v={a.escala_v} escala_w={a.escala_w} "
                               f"sesgo_w={a.sesgo_w} ruido_v={a.ruido_v} ruido_w={a.ruido_w}")

    def recibir(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        v = m.twist.twist.linear.x * self.a.escala_v + random.gauss(0, self.a.ruido_v)
        w = m.twist.twist.angular.z * self.a.escala_w + self.a.sesgo_w * (abs(m.twist.twist.linear.x) > 1e-3 or abs(m.twist.twist.angular.z) > 1e-3) \
            + random.gauss(0, self.a.ruido_w)
        if self.t_previo is not None:
            dt = t - self.t_previo
            if 0 < dt < 0.5:
                self.yaw += w * dt
                self.x += v * math.cos(self.yaw) * dt
                self.y += v * math.sin(self.yaw) * dt
        self.t_previo = t

        o = Odometry()
        o.header = m.header
        o.child_frame_id = m.child_frame_id
        o.pose.pose.position.x = self.x
        o.pose.pose.position.y = self.y
        o.pose.pose.orientation.z = math.sin(self.yaw / 2)
        o.pose.pose.orientation.w = math.cos(self.yaw / 2)
        o.twist.twist.linear.x = v
        o.twist.twist.angular.z = w
        # Covarianza de la velocidad (6x6 por filas: vx, vy, vz, wx, wy, wz).
        # El EKF pondera con esto; si las ponemos a cero, se fia ciegamente.
        cov = [0.0] * 36
        cov[0] = max(self.a.ruido_v, 0.01) ** 2 + (abs(v) * abs(self.a.escala_v - 1)) ** 2
        cov[7] = 1e-6
        cov[35] = max(self.a.ruido_w, self.a.sigma_w_min) ** 2 + (abs(w) * abs(self.a.escala_w - 1)) ** 2 + self.a.sesgo_w ** 2
        o.twist.covariance = cov
        o.pose.covariance[0] = o.pose.covariance[7] = o.pose.covariance[35] = 1e3   # no fiarse de la pose integrada
        self.pub.publish(o)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--entrada", default="/odom")
    ap.add_argument("--salida", default="/odom_degradada")
    ap.add_argument("--escala-v", dest="escala_v", type=float, default=1.02)
    ap.add_argument("--escala-w", dest="escala_w", type=float, default=1.05)
    ap.add_argument("--sesgo-w", dest="sesgo_w", type=float, default=0.01)
    ap.add_argument("--ruido-v", dest="ruido_v", type=float, default=0.0)
    ap.add_argument("--ruido-w", dest="ruido_w", type=float, default=0.0)
    ap.add_argument("--semilla", type=int, default=0)
    # Suelo de la desviacion tipica declarada para el giro. Sirve para separar
    # el efecto del ruido del de la covarianza (hipotesis 4): la condicion
    # 'ruido' declara 0,05 porque ruido_w=0,05; '--sigma-w-min 0.05' sin ruido
    # declara lo mismo sin meter ruido.
    ap.add_argument("--sigma-w-min", dest="sigma_w_min", type=float, default=0.02)
    a, resto = ap.parse_known_args()
    random.seed(a.semilla)
    rclpy.init(args=resto)
    try:
        rclpy.spin(Degradador(a))
    except (KeyboardInterrupt, Exception):
        # Parada con Ctrl+C / SIGINT desde experimento.sh: sin traza.
        pass


if __name__ == "__main__":
    main()
