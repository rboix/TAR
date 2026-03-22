#!/usr/bin/env python3
"""
Nodo ROS 2: repetir_mov.py
Ejecuta un movimiento (1, 2 o 3) un número de veces indicado y dibuja
todas las trayectorias superpuestas en una gráfica XY.

Uso:
  ros2 run p3_pkg repetir_mov <1|2|3> <num_repeticiones>

Ejemplo:
  ros2 run p3_pkg repetir_mov 2 10
"""

import sys
import os
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np



# Directorio de salida                                               
DIRECTORIO = '/workspace/ros2_ws/src/p3_pkg/dibujos'

TITULOS = {
    1: 'Triángulo equilátero (lado 3 m)',
    2: 'Cuadrado (lado 1 m)',
    3: 'Infinito (∞)',
}


def euler_from_quaternion(x, y, z, w):
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)


class RepetirMovimiento(Node):

    def __init__(self, mov: int, repeticiones: int):
        super().__init__('repetir_movimiento')

        self.mov = mov
        self.repeticiones = repeticiones

        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self._odom_callback, 10)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_received = False

        self.VEL_LINEAL = 0.15
        self.VEL_ANGULAR = 0.2

        # Lista de trayectorias: cada elemento es una lista de (x, y)
        self.trayectorias = []
        self._tray_actual = []
        self._grabando = False

        # Posiciones finales de cada repetición para análisis
        self.posiciones_finales = []

        self.get_logger().info(
            f'Nodo repetir_mov: movimiento {mov}, {repeticiones} repeticiones.')


    def _odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        self.odom_received = True

        if self._grabando:
            self._tray_actual.append((self.x, self.y))

    def _wait_odom(self):
        while not self.odom_received:
            rclpy.spin_once(self, timeout_sec=0.1)

    def _iniciar_grabacion(self):
        self._tray_actual = []
        self._grabando = True

    def _detener_grabacion(self):
        self._grabando = False
        self.trayectorias.append(list(self._tray_actual))


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

    # Movimiento básico de giro
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

    
    # Movimientos específicos según n

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

        self._parar()
        self.get_logger().info('Movimiento 3 completado.')


    def ejecutar(self):
        movimientos = {1: self.movimiento_1, 2: self.movimiento_2, 3: self.movimiento_3}
        fn = movimientos[self.mov]

        self._wait_odom()

        for i in range(self.repeticiones):
            self.get_logger().info(
                f'--- Repetición {i + 1}/{self.repeticiones} ---')
            self._iniciar_grabacion()
            fn()
            self._detener_grabacion()

            x_fin, y_fin = self.x, self.y
            self.posiciones_finales.append((x_fin, y_fin))
            dist_origen = math.hypot(x_fin, y_fin)
            self.get_logger().info(
                f'Fin repetición {i + 1}: '
                f'x={x_fin:.3f} m, y={y_fin:.3f} m, '
                f'distancia al origen={dist_origen:.3f} m')

        self.guardar_grafica()


    # Gráfica de todas las trayectorias superpuestas y análisis de error
    def guardar_grafica(self):
        if not self.trayectorias:
            self.get_logger().warn('No hay trayectorias que dibujar.')
            return

        os.makedirs(DIRECTORIO, exist_ok=True)
        ruta = os.path.join(DIRECTORIO, f'repeticion_{self.mov}.png')

        fig, axes = plt.subplots(1, 2, figsize=(16, 7))
        titulo = TITULOS.get(self.mov, f'Movimiento {self.mov}')
        fig.suptitle(
            f'Movimiento {self.mov} — {titulo} '
            f'({self.repeticiones} repeticiones)',
            fontsize=14)

        # Colores degradados para distinguir cada repetición
        colores = cm.plasma(np.linspace(0.1, 0.9, len(self.trayectorias)))

        # Gráfica izquierda: trayectorias superpuestas
        ax1 = axes[0]
        for i, (tray, color) in enumerate(zip(self.trayectorias, colores)):
            if not tray:
                continue
            xs = [p[0] for p in tray]
            ys = [p[1] for p in tray]
            ax1.plot(xs, ys, color=color, linewidth=1.2,
                     label=f'Rep. {i + 1}', alpha=0.85)
            # Marcar inicio y fin de cada repetición
            ax1.plot(xs[0],  ys[0],  'o', color=color, markersize=5)
            ax1.plot(xs[-1], ys[-1], 's', color=color, markersize=5)

        # Marcar el origen claramente
        ax1.plot(0, 0, 'k*', markersize=14, zorder=5, label='Origen (0,0)')

        ax1.set_xlabel('x (m)', fontsize=11)
        ax1.set_ylabel('y (m)', fontsize=11)
        ax1.set_title('Trayectorias superpuestas', fontsize=12)
        ax1.set_aspect('equal')
        ax1.grid(True, linestyle='--', alpha=0.5)
        ax1.legend(fontsize=7, ncol=2)

        # Margen visual
        all_x = [p[0] for t in self.trayectorias for p in t]
        all_y = [p[1] for t in self.trayectorias for p in t]
        margin = max(max(all_x) - min(all_x),
                     max(all_y) - min(all_y), 0.1) * 0.12
        ax1.set_xlim(min(all_x) - margin, max(all_x) + margin)
        ax1.set_ylim(min(all_y) - margin, max(all_y) + margin)

        # Gráfica derecha: distancia al origen por repetición
        ax2 = axes[1]
        reps = list(range(1, len(self.posiciones_finales) + 1))
        distancias = [math.hypot(x, y) for x, y in self.posiciones_finales]
        ax2.bar(reps, distancias, color=colores, edgecolor='black', linewidth=0.5)
        ax2.axhline(0, color='black', linewidth=0.8, linestyle='--')
        ax2.set_xlabel('Repetición', fontsize=11)
        ax2.set_ylabel('Distancia al origen (m)', fontsize=11)
        ax2.set_title('Error de cierre por repetición', fontsize=12)
        ax2.set_xticks(reps)
        ax2.grid(True, axis='y', linestyle='--', alpha=0.5)

        # Anotar el valor encima de cada barra
        for rep, dist in zip(reps, distancias):
            ax2.text(rep, dist + max(distancias) * 0.02,
                     f'{dist:.3f}', ha='center', va='bottom', fontsize=8)

        plt.tight_layout()
        plt.savefig(ruta, dpi=150)
        self.get_logger().info(f'Gráfica guardada en: {ruta}')
        plt.close(fig)


def main(args=None):
    try:
        mov = int(sys.argv[1])
        reps = int(sys.argv[2])
    except ValueError:
        print('Error: ambos argumentos deben ser enteros.')
        sys.exit(1)

    if mov not in (1, 2, 3):
        print('Error: movimiento debe ser 1, 2 o 3.')
        sys.exit(1)

    if reps < 1:
        print('Error: el número de repeticiones debe ser >= 1.')
        sys.exit(1)

    rclpy.init(args=args)
    nodo = RepetirMovimiento(mov, reps)
    nodo._wait_odom()

    try:
        nodo.ejecutar()
    except KeyboardInterrupt:
        nodo.get_logger().info('Interrumpido por el usuario.')
        nodo.guardar_grafica()
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()