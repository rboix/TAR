#!/usr/bin/env python3
"""
res_maze.py  –  Stop-and-turn wall follower (right-hand rule)

Lógica:
  1. FIND_WALL   → avanza recto hasta encontrar pared a la derecha o al frente
  2. FOLLOW_WALL → avanza recto manteniendo pared a la derecha
                   si aparece pared al frente → TURN_LEFT
                   si desaparece pared derecha → TURN_RIGHT
  3. TURN_LEFT   → gira en estático a la izquierda hasta que el frente quede libre
  4. TURN_RIGHT  → gira en estático a la derecha hasta detectar pared derecha
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math


# ──────────────────────────────────────────────
#  Parámetros ajustables
# ──────────────────────────────────────────────
LINEAR_SPEED  = 0.20   # m/s  – velocidad de avance
ANGULAR_SPEED = 0.30   # rad/s – velocidad de giro estático (lento = más preciso)

WALL_DIST     = 0.40   # m – distancia para considerar "hay pared"
FRONT_DIST    = 0.40   # m – distancia mínima al frente para seguir avanzando

# Sectores angulares del LiDAR
FRONT_ANGLES  = list(range(0, 20)) + list(range(340, 360))
LEFT_ANGLES   = list(range(60, 120))
RIGHT_ANGLES  = list(range(250, 290))

# Estados
FIND_WALL  = 'FIND_WALL'
FOLLOW     = 'FOLLOW'
TURN_LEFT  = 'TURN_LEFT'
TURN_RIGHT = 'TURN_RIGHT'


class MazeSolver(Node):

    def __init__(self):
        super().__init__('res_maze')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.ranges = []
        self.state  = FIND_WALL
        self.get_logger().info("MazeSolver iniciado – buscando pared...")

    # ──────────────────────────────────────────
    #  Utilidades LiDAR
    # ──────────────────────────────────────────

    def get_distance(self, angles: list) -> float:
        if not self.ranges:
            return float('inf')
        valid = []
        for i in angles:
            r = self.ranges[i % len(self.ranges)]
            if not math.isnan(r) and not math.isinf(r) and r > 0.0:
                valid.append(r)
        return min(valid) if valid else float('inf')

    # ──────────────────────────────────────────
    #  Comandos
    # ──────────────────────────────────────────

    def cmd(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x  = linear
        msg.angular.z = angular
        self.pub.publish(msg)

    def stop(self):
        self.pub.publish(Twist())

    # ──────────────────────────────────────────
    #  Callback principal
    # ──────────────────────────────────────────

    def scan_callback(self, msg: LaserScan):
        self.ranges = list(msg.ranges)

        front = self.get_distance(FRONT_ANGLES)
        right = self.get_distance(RIGHT_ANGLES)
        left  = self.get_distance(LEFT_ANGLES)

        self.get_logger().info(
            f"[{self.state}]  F:{front:.2f}  R:{right:.2f}  L:{left:.2f}",
            throttle_duration_sec=0.5
        )

        if   self.state == FIND_WALL:   self.do_find_wall(front, right)
        elif self.state == FOLLOW:      self.do_follow(front, right)
        elif self.state == TURN_LEFT:   self.do_turn_left(front, right)
        elif self.state == TURN_RIGHT:  self.do_turn_right(right)

    # ──────────────────────────────────────────
    #  FIND_WALL: avanza recto hasta topar
    # ──────────────────────────────────────────

    def do_find_wall(self, front: float, right: float):
        if front < FRONT_DIST:
            # Pared al frente → parar y girar izquierda
            self.get_logger().info("Pared al frente → TURN_LEFT")
            self.stop()
            self.state = TURN_LEFT
        elif right < WALL_DIST:
            # Pared a la derecha → empezar a seguirla
            self.get_logger().info("Pared derecha encontrada → FOLLOW")
            self.stop()
            self.state = FOLLOW
        else:
            # Seguir recto
            self.cmd(LINEAR_SPEED, 0.0)

    # ──────────────────────────────────────────
    #  FOLLOW: avanza recto pegado a la pared der
    # ──────────────────────────────────────────

    def do_follow(self, front: float, right: float):
        if front < FRONT_DIST:
            # Pared al frente → parar y girar izquierda
            self.get_logger().info("↺ Pared al frente → TURN_LEFT")
            self.stop()
            self.state = TURN_LEFT

        elif right > WALL_DIST * 2.0:
            # Pared derecha desaparece → esquina exterior → girar derecha
            self.get_logger().info("→ Esquina exterior → TURN_RIGHT")
            self.stop()
            self.state = TURN_RIGHT

        else:
            # Avanzar recto
            self.cmd(LINEAR_SPEED, 0.0)

    # ──────────────────────────────────────────
    #  TURN_LEFT: giro estático izquierda
    #  Sale cuando el frente está libre Y hay
    #  pared a la derecha (bien orientado)
    # ──────────────────────────────────────────

    def do_turn_left(self, front: float, right: float):
        if front > FRONT_DIST and right < WALL_DIST * 2.0:
            self.get_logger().info("Giro izq completado → FOLLOW")
            self.stop()
            self.state = FOLLOW
        else:
            self.cmd(0.0, ANGULAR_SPEED)

    # ──────────────────────────────────────────
    #  TURN_RIGHT: giro estático derecha
    #  Sale cuando vuelve a detectar pared derecha
    # ──────────────────────────────────────────

    def do_turn_right(self, right: float):
        if right < WALL_DIST * 1.5:
            self.get_logger().info("Giro der completado → FOLLOW")
            self.stop()
            self.state = FOLLOW
        else:
            self.cmd(0.0, -ANGULAR_SPEED)


def main():
    rclpy.init()
    node = MazeSolver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
        node.get_logger().info("Nodo detenido.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()