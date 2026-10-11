#!/usr/bin/env python3
"""Republica la IMU del simulador como la de un robot real.

La IMU de Gazebo del TurtleBot 4 es perfecta: ruido 0 y covarianza 0. Si se
fusiona tal cual, el EKF tiene un giroscopo ideal y el experimento sale mejor
de lo que saldria nunca fuera del simulador. Este nodo:

  - pone frame_id = imu_link (Gazebo manda 'turtlebot4/imu_link/imu', que no
    existe en el arbol TF y robot_localization descartaria las medidas);
  - mete ruido blanco gaussiano y un sesgo constante en el giro (rad/s);
  - declara covarianzas coherentes con ese ruido (no con el sesgo, que un
    sensor real desconoce);
  - marca la orientacion como no disponible (covarianza -1): Gazebo la manda
    perfecta y seria hacer trampa.

    python3 imu_realista.py --ruido-w 0.0015 --sesgo-w 0.0

0,0015 rad/s por muestra es del orden de un giroscopo MEMS de consumo
(densidad de ruido ~0,014 grados/s/sqrt(Hz) a ~54 Hz).
"""
import argparse
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class ImuRealista(Node):
    def __init__(self, a):
        super().__init__("imu_realista")
        self.a = a
        self.pub = self.create_publisher(Imu, a.salida, qos_profile_sensor_data)
        self.create_subscription(Imu, a.entrada, self.recibir, qos_profile_sensor_data)
        self.get_logger().info(f"{a.entrada} -> {a.salida}  frame={a.frame} ruido_w={a.ruido_w} "
                               f"sesgo_w={a.sesgo_w} ruido_a={a.ruido_a}")

    def recibir(self, m):
        m.header.frame_id = self.a.frame
        m.orientation_covariance = [-1.0] + [0.0] * 8
        g = m.angular_velocity
        g.x += random.gauss(0, self.a.ruido_w)
        g.y += random.gauss(0, self.a.ruido_w)
        g.z += random.gauss(0, self.a.ruido_w) + self.a.sesgo_w
        # Solo el ruido: un driver real no conoce su sesgo (si lo conociera, lo
        # restaria). El EKF no se entera de que existe.
        var_w = max(self.a.ruido_w, 1e-4) ** 2
        m.angular_velocity_covariance = [var_w, 0.0, 0.0, 0.0, var_w, 0.0, 0.0, 0.0, var_w]
        acc = m.linear_acceleration
        acc.x += random.gauss(0, self.a.ruido_a)
        acc.y += random.gauss(0, self.a.ruido_a)
        acc.z += random.gauss(0, self.a.ruido_a)
        var_a = max(self.a.ruido_a, 1e-3) ** 2
        m.linear_acceleration_covariance = [var_a, 0.0, 0.0, 0.0, var_a, 0.0, 0.0, 0.0, var_a]
        self.pub.publish(m)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--entrada", default="/imu_gz")
    ap.add_argument("--salida", default="/imu")
    ap.add_argument("--frame", default="imu_link")
    ap.add_argument("--ruido-w", dest="ruido_w", type=float, default=0.0015)
    ap.add_argument("--sesgo-w", dest="sesgo_w", type=float, default=0.0)
    ap.add_argument("--ruido-a", dest="ruido_a", type=float, default=0.02)
    ap.add_argument("--semilla", type=int, default=1)
    a, resto = ap.parse_known_args()
    random.seed(a.semilla)
    rclpy.init(args=resto)
    try:
        rclpy.spin(ImuRealista(a))
    except (KeyboardInterrupt, Exception):
        # Parada con Ctrl+C / SIGINT desde experimento.sh: sin traza.
        pass


if __name__ == "__main__":
    main()
