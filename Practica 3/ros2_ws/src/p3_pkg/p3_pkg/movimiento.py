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


class MovimientoRobot(Node):
    """Nodo que controla el movimiento del robot mediante velocidades."""

    def __init__(self, controlador=0):
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
        self.VEL_LINEAL = 0.08   # m/s
        self.VEL_ANGULAR = 0.2   # rad/s

        self.controlador = controlador

        # ── Parámetros de corrección de giro (ajustables en tiempo de ejecución) ──
        # Factor de escala: si el robot gira de más → valor < 1.0
        #                   si el robot gira de menos → valor > 1.0
        # Velocidad angular usada en girar_alternativo
        self.declare_parameter('angular_vel', 0.2)

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

    def avanzar(self, distancia: float):
        """Selecciona el método de avance según el controlador."""
        if self.controlador == 1:
            return self.avanzar_alternativo(distancia)
        else:
            return self.avanzar_normal(distancia)

    # Movimiento básico de avanzar                                        
    def avanzar_normal(self, distancia: float):
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


    def avanzar_alternativo(self, distancia: float):
        """
        Avanza una distancia dada (metros) en línea recta,
        corrigiendo la orientación en tiempo real.
        """
        self._wait_odom()
        rclpy.spin_once(self, timeout_sec=0.1)

        x0, y0 = self.x, self.y
        yaw_objetivo = self.yaw  # Mantener orientación inicial

        msg = Twist()
        recorrido = 0.0

        Kp_ang = 2.0  # Ganancia proporcional para corrección angular

        self.get_logger().info(f'Avanzando {distancia:.2f} m con corrección...')

        while recorrido < distancia:
            rclpy.spin_once(self, timeout_sec=0.05)

            # Distancia recorrida
            recorrido = math.hypot(self.x - x0, self.y - y0)

            # Error angular (normalizado)
            error_yaw = math.atan2(
                math.sin(yaw_objetivo - self.yaw),
                math.cos(yaw_objetivo - self.yaw)
            )

            # Control
            msg.linear.x = self.VEL_LINEAL
            msg.angular.z = Kp_ang * error_yaw

            self.pub_vel.publish(msg)

        self._parar()
        time.sleep(0.3)
    

    # Movimiento básico de giro
    # Si se pasa un 1 como argumento al nodo, se usará un controlador proporcional (P) para el giro
    def girar(self, angulo: float):
        if self.controlador == 1:
            return self.girar_alternativo(angulo)
        else:
            return self.girar_normal(angulo)
        
    
    def girar_normal(self, angulo: float):
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


    # Gira un ángulo dado (radianes) usando un controlador proporcional (P).
    # La velocidad angular es proporcional al error restante, lo que reduce el sobreimpulso y mejora la precisión del giro.
    def girar_alternativo(self, angulo: float):
        self._wait_odom()
        rclpy.spin_once(self, timeout_sec=0.1)

        angular_vel = self.get_parameter('angular_vel').value
        # Ya no aplicamos angular_scale al objetivo

        yaw0 = self.yaw
        objetivo = math.atan2(
            math.sin(yaw0 + angulo),
            math.cos(yaw0 + angulo))

        KP = 3.0       # Ganancia un poco mayor para respuesta más ágil
        VEL_MIN = 0.05 # Suficiente para vencer la inercia

        msg = Twist()
        while True:
            rclpy.spin_once(self, timeout_sec=0.05)
            error = math.atan2(
                math.sin(objetivo - self.yaw),
                math.cos(objetivo - self.yaw))
            if abs(error) < 0.01:   # ~0.57°
                break
            vel = max(VEL_MIN, min(angular_vel, KP * abs(error)))
            msg.angular.z = vel if error > 0 else -vel
            self.pub_vel.publish(msg)

        self._parar()
        time.sleep(0.3)

    # Parar el robot
    def _parar(self):
        """Publica velocidad cero varias veces para asegurar parada completa."""
        for _ in range(5):
            self.pub_vel.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.05)
        time.sleep(0.2)

    
    # Movimientos específicos según n

    # Avanzar 2 metros
    def movimiento_0(self):
        self.get_logger().info('Movimiento 0: avance lineal 2 m')
        self.avanzar(2.0)
        self.get_logger().info('Movimiento 0 completado.')

    # Triángulo equilátero de lado 3 m
    def movimiento_1(self):
        self.get_logger().info('Movimiento 1: triángulo equilátero 3 m')
        angulo_exterior = math.radians(120)  # 2π/3

        # El triángulo se forma avanzando 3 m y girando 120° tres veces
        for i in range(3):
            self.avanzar(3.0)
            self.girar(angulo_exterior)
        self.get_logger().info('Movimiento 1 completado.')

    # Cuadrado de lado 1 m
    def movimiento_2(self):
        self.get_logger().info('Movimiento 2: cuadrado 1 m')
        angulo = math.radians(90)

        # El cuadrado se forma avanzando 1 m y girando 90° cuatro veces
        for i in range(4):
            self.avanzar(1.0)
            self.girar(angulo)
        self.get_logger().info('Movimiento 2 completado.')

    # Figura en forma de infinito
    def movimiento_3(self):
        self.get_logger().info('Movimiento 3: infinito')

        distancia_uno = 0.5     
        distancia_grande = 1   

        angulo_uno = math.radians(120)  
        angulo_dos = math.radians(240)

        self.avanzar(distancia_uno)         # Avanzo 0.5m
        self.girar(angulo_uno)              # Giro de 120º

        self.avanzar(distancia_grande)      # Avanzo 1m
        self.girar(angulo_dos)              # Giro de 240º

        self.avanzar(distancia_uno)         # Avanzo 0.5m
        self.girar(angulo_dos)              # Giro de 240º

        self.avanzar(distancia_grande)      # Avanzo 1m
        self.girar(angulo_uno)              # Giro de 120º

        self._parar()
        self.get_logger().info('Movimiento 3 completado.')



def main(args=None):
    try:
        n = int(sys.argv[1])
        controlador = int(sys.argv[2]) # Si se pasa un 1 como segundo argumento, se usará otra función de girar
    except ValueError:
        print('Error: el argumento debe ser un entero (0, 1, 2 o 3).')
        sys.exit(1)

    if n not in (0, 1, 2, 3):
        print('Error: movimiento no reconocido. Valores válidos: 0, 1, 2, 3.')
        sys.exit(1)

    rclpy.init(args=args)
    nodo = MovimientoRobot(controlador=controlador)

    # Esperar a que el nodo esté suscrito y reciba odometría
    nodo.get_logger().info('Esperando odometría...')
    nodo._wait_odom()

    movimientos = {
        0: nodo.movimiento_0,
        1: nodo.movimiento_1,
        2: nodo.movimiento_2,
        3: nodo.movimiento_3,
    }

    movimientos[n]()

    nodo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()