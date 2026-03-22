#!/usr/bin/env python3

import sys
import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import string


TITULOS = {
    0: 'Movimiento 0 — Línea recta (2 m)',
    1: 'Movimiento 1 — Triángulo equilátero (lado 3 m)',
    2: 'Movimiento 2 — Cuadrado (lado 1 m)',
    3: 'Movimiento 3 — Infinito (∞)',
}

DIRECTORIO = '/workspace/ros2_ws/src/p3_pkg/dibujos'


class DibujaMovimiento(Node):
    """Suscriptor de odometría que acumula posiciones (x, y) y las dibuja."""

    def __init__(self, n: int):
        super().__init__('dibuja_movimiento')
        self.n = n
        self.xs = []
        self.ys = []

        self.sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, 10)

        self.get_logger().info(
            f'Nodo dibuja_mov activo (movimiento {n}). '
            'Suscrito a /odom. Pulsa Ctrl+C para guardar la gráfica.')

    def _odom_callback(self, msg: Odometry):
        self.xs.append(msg.pose.pose.position.x)
        self.ys.append(msg.pose.pose.position.y)

    def guardar_grafica(self):
        if not self.xs:
            self.get_logger().warn('No se recibieron datos de odometría.')
            return

        os.makedirs(DIRECTORIO, exist_ok=True)
        ruta = os.path.join(DIRECTORIO, f'trayectoria_{self.n}.png')

        fig, ax = plt.subplots(figsize=(7, 7))

        ax.plot(self.xs, self.ys, 'b-', linewidth=1.8, label='Trayectoria')
        ax.plot(self.xs[0],  self.ys[0],  'go', markersize=10, label='Inicio')
        ax.plot(self.xs[-1], self.ys[-1], 'rs', markersize=10, label='Fin')

        # Margen visual para que la figura no quede pegada a los ejes
        # De esta forma podemos ver claramente la forma del dibujo
        x_range = max(abs(max(self.xs) - min(self.xs)), 0.1)
        y_range = max(abs(max(self.ys) - min(self.ys)), 0.1)
        margin = max(x_range, y_range) * 0.15
        ax.set_xlim(min(self.xs) - margin, max(self.xs) + margin)
        ax.set_ylim(min(self.ys) - margin, max(self.ys) + margin)

        # Configuración de la gráfica
        ax.set_aspect('equal')
        ax.set_xlabel('x (m)', fontsize=12)
        ax.set_ylabel('y (m)', fontsize=12)
        ax.set_title(TITULOS.get(self.n, f'Movimiento {self.n}'), fontsize=13)
        ax.grid(True, linestyle='--', alpha=0.6)
        ax.legend(fontsize=11)

        plt.tight_layout()
        plt.savefig(ruta, dpi=150)
        self.get_logger().info(f'Gráfica guardada en: {ruta}')
        plt.close(fig)


def main(args=None):

    try:
        n = int(sys.argv[1])
        ruta_guardado = int(sys.argv[2])
    except ValueError:
        print('Error: el argumento debe ser un entero (0, 1, 2 o 3).')
        sys.exit(1)
    
    if (ruta_guardado == 1):
        DIRECTORIO = '/workspace/ros2_ws/src/p3_pkg/dibujos_controlador'


    rclpy.init(args=args)
    nodo = DibujaMovimiento(n)

    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.guardar_grafica()
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()