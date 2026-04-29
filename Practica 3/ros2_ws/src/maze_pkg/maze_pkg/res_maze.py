import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math


# ──────────────────────────────────────────────
#  Parámetros ajustables
# ──────────────────────────────────────────────
LINEAR_SPEED  = 0.25  # m/s
ANGULAR_SPEED = 0.40   # rad/s

WALL_DIST     = 0.35   # distancia de pared lateral
FRONT_DIST    = 0.35   # distancia de pared frontal


FRONT_ANGLES  = list(range(0, 25)) + list(range(335, 360))
LEFT_ANGLES   = list(range(60, 120))
RIGHT_ANGLES  = list(range(240, 300))

# Estados
FIND_WALL   = 'FIND_WALL'
FOLLOW_WALL = 'FOLLOW_WALL'


class MazeSolver(Node):

    def __init__(self):
        super().__init__('res_maze')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.ranges = []
        self.state  = FIND_WALL
        self.get_logger().info("MazeSolver iniciado - buscando pared...")

    def get_distance(self, angles: list) -> float:
        """Distancia mínima válida en el sector indicado."""
        if not self.ranges:
            return float('inf')
        valid = []
        for i in angles:
            r = self.ranges[i % len(self.ranges)]
            if not math.isnan(r) and not math.isinf(r) and r > 0.0:
                valid.append(r)
        return min(valid) if valid else float('inf')

    def cmd(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x  = linear
        msg.angular.z = angular
        self.pub.publish(msg)

    def stop(self):
        self.pub.publish(Twist())

    def scan_callback(self, msg: LaserScan):
        self.ranges = list(msg.ranges)

        front = self.get_distance(FRONT_ANGLES)
        left  = self.get_distance(LEFT_ANGLES)
        right = self.get_distance(RIGHT_ANGLES)

        self.get_logger().info(
            f"[{self.state}]  F:{front:.2f}  L:{left:.2f}  R:{right:.2f}",
            throttle_duration_sec=0.5
        )

        if self.state == FIND_WALL:
            self.do_find_wall(front)
        elif self.state == FOLLOW_WALL:
            self.do_follow_wall(front, left, right)

    # Estado para buscar pared

    def do_find_wall(self, front: float):
        right = self.get_distance(RIGHT_ANGLES)

        if front < FRONT_DIST or right < WALL_DIST * 2:
            self.get_logger().info("Pared encontrada → FOLLOW_WALL")
            self.state = FOLLOW_WALL
            self.stop()
        else:
            self.cmd(LINEAR_SPEED, 0.0)

    # Estado para seguir pared

    def do_follow_wall(self, front: float, left: float, right: float):

        if front < FRONT_DIST:
            self.get_logger().info("Pared al frente - giro izquierda",
                                   throttle_duration_sec=0.5)
            self.cmd(0.0, ANGULAR_SPEED)

        elif right > WALL_DIST * 1.5:
            self.get_logger().info("Abriendo derecha curva derecha",
                                   throttle_duration_sec=0.5)
            self.cmd(LINEAR_SPEED, -ANGULAR_SPEED * 0.8)

        elif right < WALL_DIST * 0.6:
            self.get_logger().info("Muy cerca pared der - corrección izq",
                                   throttle_duration_sec=0.5)
            self.cmd(LINEAR_SPEED, ANGULAR_SPEED * 0.5)

        else:
            self.get_logger().info("Avanzando recto",
                                   throttle_duration_sec=0.5)
            self.cmd(LINEAR_SPEED, 0.0)


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