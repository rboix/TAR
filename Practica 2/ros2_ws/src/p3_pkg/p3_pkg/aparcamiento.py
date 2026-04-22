#!/usr/bin/env python3

import sys
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def euler_from_quaternion(x, y, z, w):
    """Convierte un cuaternión a ángulos de Euler (roll, pitch, yaw)."""
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return yaw


class Aparcamiento(Node):
    """Nodo que controla el movimiento del robot mediante velocidades."""

    def __init__(self):
        super().__init__('movimiento_robot')

        # Publisher al topic de velocidades
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber a odometría para obtener posición y orientación actuales
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self._odom_callback, 10)

        # Estado interno del robot
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_received = False

        # Velocidades del robot en unidades del SI
        self.VEL_LINEAL = 0.15   # m/s
        self.VEL_ANGULAR = 0.2   # rad/s

        self.get_logger().info('Nodo movimiento iniciado.')

    #  Callback de odometría                                               
    def _odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        self.odom_received = True

    def _wait_odom(self):
        """Espera hasta recibir el primer mensaje de odometría."""
        while not self.odom_received:
            rclpy.spin_once(self, timeout_sec=0.1)

    # Movimiento básico de avanzar                                        
    def avanzar(self, distancia: float):
        """
        Avanza una distancia dada (metros) en línea recta.
        Controla el progreso mediante odometría.
        """
        self._wait_odom()
        rclpy.spin_once(self, timeout_sec=0.1)

        x0, y0 = self.x, self.y
        msg = Twist()
        msg.linear.x = self.VEL_LINEAL
        recorrido = 0.0

        self.get_logger().info(f'Avanzando {distancia:.2f} m ...')
        while recorrido < distancia:
            self.pub_vel.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            recorrido = math.hypot(self.x - x0, self.y - y0)

        self._parar()
        time.sleep(0.5)
        
    
    def girar(self, angulo: float):
        """
        Gira un ángulo dado (radianes).
        Positivo = izquierda (sentido antihorario), negativo = derecha.
        """
        self._wait_odom()
        rclpy.spin_once(self, timeout_sec=0.1)

        yaw0 = self.yaw
        objetivo = yaw0 + angulo
        # Normalizar a [-π, π]
        objetivo = math.atan2(math.sin(objetivo), math.cos(objetivo))

        msg = Twist()
        msg.angular.z = self.VEL_ANGULAR if angulo > 0 else -self.VEL_ANGULAR

        self.get_logger().info(f'Girando {math.degrees(angulo):.1f}° ...')

        # Control de giro mediante odometría
        # Calculamos el error angular y corregimos hasta alcanzar el objetivo
        while True:
            rclpy.spin_once(self, timeout_sec=0.05)
            error = objetivo - self.yaw
            error = math.atan2(math.sin(error), math.cos(error))
            if abs(error) < 0.015:  
                break
            self.pub_vel.publish(msg)

        self._parar()
        time.sleep(0.5)

    # Parar el robot
    def _parar(self):
        """Publica velocidad cero varias veces para asegurar parada completa."""
        for _ in range(5):
            self.pub_vel.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.05)
        time.sleep(0.2)

    # Aparcamiento completo
    def aparcar(self):
        self.get_logger().info('Paso 1: avance lineal 1.5 m')
        self.avanzar(1.5)
        self.get_logger().info('Paso 1 completado.')

        angulo = math.radians(-90)  
        self.get_logger().info('Paso 1: giro -90º')
        self.girar(angulo)
        self.get_logger().info('Paso 2 completado.')

        self.get_logger().info('Paso 3: avance lineal 1.5 m')
        self.avanzar(1.5)
        self.get_logger().info('Paso 3 completado.')
        self.get_logger().info('APARCAMIENTO COMPLETADO.')



def main(args=None):
    rclpy.init(args=args)
    nodo = Aparcamiento()
    nodo.aparcar()
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()