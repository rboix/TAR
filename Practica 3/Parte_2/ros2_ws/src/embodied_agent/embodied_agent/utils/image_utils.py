"""Utilidades para imágenes: encoding base64 y conversión depth → 3D."""
import base64
import numpy as np


def encode_image_base64(image_bgr: np.ndarray) -> bytes:
    """Convierte imagen BGR (OpenCV) a JPEG en bytes."""
    import cv2
    _, buf = cv2.imencode('.jpg', image_bgr, [cv2.IMWRITE_JPEG_QUALITY, 85])
    return buf.tobytes()


def depth_pixel_to_3d(px: int, py: int, depth_m: float,
                       fx: float, fy: float, cx: float, cy: float
                       ) -> tuple[float, float, float]:
    """Proyecta un píxel con profundidad a coordenadas 3D en el frame cámara."""
    x = (px - cx) * depth_m / fx
    y = (py - cy) * depth_m / fy
    return (x, y, depth_m)


def camera_to_map(cam_xyz: tuple[float, float, float],
                  robot_pose: tuple[float, float, float]
                  ) -> tuple[float, float]:
    """Transforma punto 3D en frame cámara a coordenadas 2D del mapa."""
    import math
    rx, ry, rtheta = robot_pose
    cx, _, cz = cam_xyz
    # distancia y ángulo relativo al robot
    dist = math.sqrt(cx ** 2 + cz ** 2)
    angle = math.atan2(cx, cz)
    map_x = rx + dist * math.cos(rtheta + angle)
    map_y = ry + dist * math.sin(rtheta + angle)
    return (map_x, map_y)
