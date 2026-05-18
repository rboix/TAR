"""
action_executor_node (Fase 5 — variante open-loop).

Decisión arquitectónica
=======================
El TurtleBot 4 sim (turtlebot4_ignition_bringup con namespace) no
publica un árbol tf consistente: faltan los frames `odom` y `base_link`
del robot y conviven dos topics tf en paralelo (`/tf` y `/turtlebot4/tf`).
Sin ese árbol no se puede ejecutar la pipeline canónica
`bbox → tf → Nav2.NavigateToPose` que describe la sección 8 de CLAUDE.md.

En lugar de gastar horas reparando el stack del TB4, esta fase implementa
navegación open-loop sobre `/cmd_vel` + `/odom` directamente:

    bbox del Gemini → (yaw_delta, distance) en frame del robot
    rotate cerrando lazo con /odom (yaw)
    drive forward cerrando lazo con /odom (posición)

Todo lo que necesitamos lo tenemos publicando: `/turtlebot4/cmd_vel`,
`/turtlebot4/odom`, `/oakd/rgb/preview/depth` y `camera_info`.

Para portar al robot real bastaría con sustituir `_do_navigate` por la
variante Nav2 (el preprocesado del bbox sería idéntico).

Acciones soportadas
===================
  - none, ask_user        → no-op (sólo cierra lazo en /action_result)
  - rotate                → closed-loop sobre yaw de /odom
  - navigate / inspect    → orienta al objetivo + avanza a (dist-stop)
  - panoramic, investigate → stub (Fase 6/7)
"""
import json
import math
import threading
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from embodied_agent.utils.geometry import pixel_to_3d, yaw_from_quaternion


# ============================================== Constantes de control

# Velocidades nominales y rate de publicación de cmd_vel.
_ROTATE_ANGULAR_SPEED = 0.6   # rad/s
_LINEAR_SPEED = 0.25          # m/s
_CMD_VEL_RATE_HZ = 20.0

# Tolerancias de cierre de lazo.
_YAW_TOLERANCE_RAD = math.radians(3.0)   # ~3°
_DIST_TOLERANCE_M = 0.05                 # 5 cm

# Timeouts (sim-time).
_ROTATE_TIMEOUT_S = 15.0
_DRIVE_TIMEOUT_S = 30.0

# Margen extra que añadimos al stop_distance pedido por Gemini para
# compensar la incertidumbre del depth picking. Sin este margen, "para
# a 0.5 m del objeto" pone al CENTRO del robot a 0.5 m → el frente del
# robot queda a ~0.33 m (radio ~0.17 m) y a veces toca al objeto si la
# depth estimada se quedó corta.
#
# Subir este valor → robot se queda más lejos del objeto (más seguro).
# Bajarlo  → robot se acerca más (mejor para inspección).
# 0.10 m es un compromiso: el frente del robot queda a ~0.23 m del
# centro del objeto cuando Gemini pide 0.5 m de stop_distance.
_STOP_DISTANCE_MARGIN_M = 0.10

# Tope duro de cuánto se permite avanzar de una sola vez. Si la depth
# del bbox sale mal (p.ej. cogió la pared del fondo a 8 m), no
# queremos que el robot salga disparado al otro lado del laboratorio.
_MAX_DRIVE_DIST_M = 3.0

# Fallback de intrínsecos si /camera/camera_info no llega a tiempo.
_FALLBACK_INTRINSICS = {
    'fx': 250.0, 'fy': 250.0, 'cx': 150.0, 'cy': 150.0,
    'width': 300, 'height': 300,
}


def _shortest_angle_diff(a: float, b: float) -> float:
    """Distancia angular firmada de `b` a `a`, envuelta a [-pi, pi]."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


# ====================================================================

class ActionExecutorNode(Node):

    def __init__(self):
        super().__init__('action_executor_node')

        # Caches con locks ligeros.
        self._depth_lock = threading.Lock()
        self._latest_depth: Optional[np.ndarray] = None
        self._latest_depth_scale: float = 1000.0

        self._intr_lock = threading.Lock()
        self._intrinsics = dict(_FALLBACK_INTRINSICS)
        self._intrinsics_received = False

        self._odom_lock = threading.Lock()
        self._latest_pose: Optional[tuple[float, float, float]] = None

        # Lock de exclusión mutua para acciones físicas (no queremos dos
        # rotates concurrentes).
        self._busy = threading.Lock()

        # Publishers.
        self._pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self._pub_result = self.create_publisher(String, '/action_result', 10)

        # Subscriptions. Sensor data (depth / camera_info / odom) en
        # BEST_EFFORT para ser compatibles con la mayoría de drivers.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._sub_depth = self.create_subscription(
            Image, '/camera/depth', self._on_depth, sensor_qos)
        self._sub_caminfo = self.create_subscription(
            CameraInfo, '/camera/camera_info', self._on_camera_info, sensor_qos)
        self._sub_odom = self.create_subscription(
            Odometry, '/odom', self._on_odom, sensor_qos)
        self._sub_cmd = self.create_subscription(
            String, '/action_command', self._on_command, 10)

        self.get_logger().info(
            'action_executor_node arrancado (fase 5 — open-loop, '
            'sin Nav2 ni tf2)'
        )

    # ============================================================ sensores

    def _on_depth(self, msg: Image):
        try:
            arr, scale = _depth_msg_to_array(msg)
        except Exception as e:
            self.get_logger().warn(f'[exec] error decodificando depth: {e}')
            return
        with self._depth_lock:
            self._latest_depth = arr
            self._latest_depth_scale = scale

    def _on_camera_info(self, msg: CameraInfo):
        if len(msg.k) < 9:
            return
        with self._intr_lock:
            self._intrinsics = {
                'fx': float(msg.k[0]),
                'fy': float(msg.k[4]),
                'cx': float(msg.k[2]),
                'cy': float(msg.k[5]),
                'width': int(msg.width),
                'height': int(msg.height),
            }
            if not self._intrinsics_received:
                self.get_logger().info(
                    f'[exec] intrínsecos recibidos: {self._intrinsics}')
                self._intrinsics_received = True

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        with self._odom_lock:
            self._latest_pose = (float(p.x), float(p.y), float(yaw))

    # ============================================================== comando

    def _on_command(self, msg: String):
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(
                f'[exec] /action_command no es JSON válido: {e}')
            return
        action = cmd.get('action', 'none')
        self.get_logger().info(f'[exec] comando: {cmd}')

        # Atajos síncronos.
        if action in ('none', 'ask_user'):
            self._publish_result(action, 'succeeded',
                                 'no se requiere acción física')
            return
        if action in ('panoramic', 'investigate'):
            self._publish_result(
                action, 'not_implemented_yet',
                f'la acción "{action}" se implementa en una fase posterior')
            return

        if not self._busy.acquire(blocking=False):
            self._publish_result(
                action, 'skipped', 'executor ocupado con otra acción')
            return
        threading.Thread(
            target=self._run_action_safely, args=(action, cmd), daemon=True,
        ).start()

    def _run_action_safely(self, action: str, cmd: dict):
        try:
            if action == 'rotate':
                self._do_rotate(cmd)
            elif action in ('navigate', 'inspect'):
                self._do_navigate(cmd)
            else:
                self._publish_result(action, 'failed',
                                     f'acción desconocida: {action}')
        except Exception as e:
            self.get_logger().error(
                f'[exec] excepción ejecutando {action}: {e}')
            self._publish_result(action, 'failed', f'excepción: {e}')
        finally:
            self._busy.release()

    # =============================================================== rotate

    def _do_rotate(self, cmd: dict):
        """Giro absoluto: `degrees` grados respecto al yaw actual."""
        degrees = cmd.get('degrees')
        if degrees is None:
            self._publish_result('rotate', 'failed',
                                 'rotate requiere "degrees"')
            return
        delta_rad = math.radians(float(degrees))
        with self._odom_lock:
            if self._latest_pose is None:
                self._publish_result('rotate', 'failed', 'sin /odom')
                return
            current_yaw = self._latest_pose[2]
        target_yaw = current_yaw + delta_rad
        self.get_logger().info(
            f'[exec] rotate {degrees:.1f}° (closed-loop) → '
            f'target_yaw={math.degrees(target_yaw):.1f}°'
        )
        if self._rotate_to_yaw(target_yaw, timeout_s=_ROTATE_TIMEOUT_S):
            self._publish_result('rotate', 'succeeded',
                                 f'rotación de {degrees:.1f}° completada')
        else:
            self._publish_result('rotate', 'failed',
                                 'timeout cerrando lazo de rotación')

    # ============================================================ navigate

    def _do_navigate(self, cmd: dict):
        """Navegación open-loop: orientar al bbox + avanzar."""
        bbox = cmd.get('image_bbox')
        target_label = cmd.get('target') or 'objetivo'
        stop_distance = float(cmd.get('distance') or 0.5)
        action_name = cmd.get('action', 'navigate')

        if not bbox or len(bbox) != 4:
            self._publish_result(
                action_name, 'failed',
                'navigate requiere image_bbox [x0,y0,x1,y1]')
            return

        # 1) Snapshot de sensores.
        with self._depth_lock:
            depth = self._latest_depth
            depth_scale = self._latest_depth_scale
        if depth is None:
            self._publish_result(
                action_name, 'failed',
                'sin imagen de profundidad disponible aún')
            return

        with self._intr_lock:
            fx, fy = self._intrinsics['fx'], self._intrinsics['fy']
            cx, cy = self._intrinsics['cx'], self._intrinsics['cy']
            intr_w = self._intrinsics.get('width') or 0
            intr_h = self._intrinsics.get('height') or 0
            intr_received = self._intrinsics_received

        # Si los intrínsecos vienen de una cámara de distinta resolución
        # que la depth, reescalamos. Con depth alineada al RGB preview
        # estos factores son 1.0 y no pasa nada.
        h_d, w_d = depth.shape[:2]
        sx = (w_d / float(intr_w)) if intr_w > 0 else 1.0
        sy = (h_d / float(intr_h)) if intr_h > 0 else 1.0
        fx_d, fy_d = fx * sx, fy * sy
        cx_d, cy_d = cx * sx, cy * sy

        xyz = pixel_to_3d(bbox, depth, fx_d, fy_d, cx_d, cy_d,
                          depth_scale=depth_scale)
        if xyz is None:
            self._publish_result(
                action_name, 'failed',
                'no hay profundidad fiable en el bbox del objetivo')
            return
        x_cam, _y_cam, z_cam = xyz

        # 2) bbox 3D → (yaw_delta, dist) en frame del robot.
        # Camera optical:  z forward, x right.
        # Robot REP-103:   x forward, y left, yaw+ = giro a izquierda.
        # Para apuntar al objeto (objeto a la derecha => giro a derecha):
        #     yaw_delta = -atan2(x_cam, z_cam)
        # La distancia horizontal a recorrer es sqrt(x² + z²); ignoramos
        # la altura del objeto (y_cam) — el robot solo se mueve en xy.
        yaw_delta = -math.atan2(x_cam, z_cam)
        horizontal_dist = math.hypot(x_cam, z_cam)
        # Stop a `stop_distance` del objeto + un margen para el radio
        # físico del robot y la incertidumbre del depth. Cap superior
        # defensivo para no salir disparado si la depth es errónea.
        effective_stop = stop_distance + _STOP_DISTANCE_MARGIN_M
        drive_dist = max(0.0, horizontal_dist - effective_stop)
        drive_dist = min(drive_dist, _MAX_DRIVE_DIST_M)

        intr_note = ' (intrínsecos por defecto)' if not intr_received else ''
        self.get_logger().info(
            f'[exec] navigate → "{target_label}": '
            f'cam(x={x_cam:.2f}, z={z_cam:.2f}) → '
            f'yaw_delta={math.degrees(yaw_delta):.1f}°, '
            f'dist={horizontal_dist:.2f}m, drive={drive_dist:.2f}m '
            f'(stop_at={stop_distance:.2f}m + margen {_STOP_DISTANCE_MARGIN_M:.2f}m)'
            f'{intr_note}'
        )

        # 3) Snapshot pose inicial.
        with self._odom_lock:
            if self._latest_pose is None:
                self._publish_result(
                    action_name, 'failed', 'sin /odom disponible aún')
                return
            _, _, start_yaw = self._latest_pose
        target_yaw = start_yaw + yaw_delta

        # 4) Orientar al objetivo (sólo si el ángulo merece la pena).
        if abs(yaw_delta) > _YAW_TOLERANCE_RAD:
            if not self._rotate_to_yaw(target_yaw, timeout_s=_ROTATE_TIMEOUT_S):
                self._publish_result(
                    action_name, 'failed',
                    'timeout cerrando lazo de rotación inicial')
                return

        # 5) Avanzar.
        if drive_dist > _DIST_TOLERANCE_M:
            if not self._drive_forward(drive_dist, timeout_s=_DRIVE_TIMEOUT_S):
                self._publish_result(
                    action_name, 'failed',
                    'timeout cerrando lazo de avance')
                return

        self._publish_result(
            action_name, 'succeeded',
            f'aproximación a "{target_label}" completada')

    # ======================================================= control loops

    def _rotate_to_yaw(self, target_yaw: float, timeout_s: float) -> bool:
        """Closed-loop: gira hasta que |yaw_target − yaw_actual| < tol.

        Devuelve True si convergió, False si saltó el timeout.
        El timeout se mide en sim-time para tolerar Gazebos con RTF < 1.
        """
        clock = self.get_clock()
        end_time = clock.now() + Duration(seconds=timeout_s)
        period_s = 1.0 / _CMD_VEL_RATE_HZ
        twist = Twist()
        while rclpy.ok() and clock.now() < end_time:
            with self._odom_lock:
                if self._latest_pose is None:
                    return False
                current_yaw = self._latest_pose[2]
            err = _shortest_angle_diff(target_yaw, current_yaw)
            if abs(err) < _YAW_TOLERANCE_RAD:
                self._pub_cmd_vel.publish(Twist())
                return True
            # Velocidad: máxima si lejos, suavizada cuando estás cerca
            # del target para evitar overshoot.
            speed = _ROTATE_ANGULAR_SPEED
            if abs(err) < 0.4:  # ~23°
                speed = max(0.15, _ROTATE_ANGULAR_SPEED * (abs(err) / 0.4))
            twist.angular.z = math.copysign(speed, err)
            self._pub_cmd_vel.publish(twist)
            time.sleep(period_s)
        # Timeout: frenamos.
        self._pub_cmd_vel.publish(Twist())
        return False

    def _drive_forward(self, target_dist: float, timeout_s: float) -> bool:
        """Closed-loop: avanza target_dist metros desde la pose actual.

        Asume que ya estamos apuntando al objetivo. Solo `linear.x > 0`,
        no corrige deriva angular durante el avance (suficiente para
        distancias cortas en el demo).
        """
        clock = self.get_clock()
        end_time = clock.now() + Duration(seconds=timeout_s)
        period_s = 1.0 / _CMD_VEL_RATE_HZ

        with self._odom_lock:
            if self._latest_pose is None:
                return False
            start_x, start_y, _ = self._latest_pose

        twist = Twist()
        while rclpy.ok() and clock.now() < end_time:
            with self._odom_lock:
                if self._latest_pose is None:
                    return False
                cur_x, cur_y, _ = self._latest_pose
            traveled = math.hypot(cur_x - start_x, cur_y - start_y)
            remaining = target_dist - traveled
            if remaining < _DIST_TOLERANCE_M:
                self._pub_cmd_vel.publish(Twist())
                return True
            # Frena al acercarse para no rebasar.
            speed = _LINEAR_SPEED
            if remaining < 0.3:
                speed = max(0.05, _LINEAR_SPEED * (remaining / 0.3))
            twist.linear.x = speed
            self._pub_cmd_vel.publish(twist)
            time.sleep(period_s)
        self._pub_cmd_vel.publish(Twist())
        return False

    # ============================================================== output

    def _publish_result(self, action: str, status: str, message: str):
        msg = String()
        msg.data = json.dumps({
            'action': action, 'status': status, 'message': message,
        }, ensure_ascii=False)
        self._pub_result.publish(msg)
        log = self.get_logger()
        if status == 'succeeded':
            log.info(f'[exec] resultado: {msg.data}')
        elif status == 'not_implemented_yet':
            log.warn(f'[exec] resultado: {msg.data}')
        else:
            log.error(f'[exec] resultado: {msg.data}')


# ===================================================== helpers de imagen

def _depth_msg_to_array(msg: Image) -> tuple[np.ndarray, float]:
    """Convierte sensor_msgs/Image (depth) a (np.array 2D, scale).

    Soporta los encodings habituales en OAK-D y Gazebo:
      - 16UC1 / mono16   → mm  (scale=1000)
      - 32FC1            → m   (scale=1)
    """
    enc = (msg.encoding or '').lower()
    if enc == '32fc1':
        dtype = np.float32
        scale = 1.0
    else:
        dtype = np.uint16
        scale = 1000.0
    arr = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width)
    return arr, scale


# ====================================================================

def main(args=None):
    rclpy.init(args=args)
    node = ActionExecutorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
