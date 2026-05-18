"""Helpers de geometría usados por action_executor_node.

En esta fase (5 — open-loop) sólo necesitamos:
  - `yaw_from_quaternion`: extraer el yaw 2D de la orientación de /odom.
  - `pixel_to_3d`: proyectar el centro del bbox de Gemini a coordenadas
    (x, y, z) en el frame óptico de la cámara usando la imagen de
    profundidad.

Funciones de soporte para Nav2 (`make_pose_stamped`, `build_navigate_goal`)
se eliminaron al pasar a navegación open-loop. Si más adelante se monta
Nav2 en el robot real, vivirán de nuevo aquí.
"""
import math
from typing import Optional

import numpy as np
from geometry_msgs.msg import Quaternion


# ----------------------------------------------------------- quaterniones

def yaw_from_quaternion(q: Quaternion) -> float:
    """Yaw (rotación alrededor de z) en radianes desde un Quaternion 2D."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


# ----------------------------------------------------- bbox → 3D en cámara

def pixel_to_3d(bbox: list[float],
                depth_image: np.ndarray,
                fx: float, fy: float, cx: float, cy: float,
                depth_scale: float = 1000.0,
                depth_percentile: float = 10.0,
                ) -> Optional[tuple[float, float, float]]:
    """Proyecta el centro del bbox a (x, y, z) en el frame óptico de la cámara.

    Args:
        bbox: [x_min, y_min, x_max, y_max] normalizado a [0,1].
        depth_image: array 2D (H, W) con la profundidad cruda.
        fx, fy, cx, cy: intrínsecos pinhole en píxeles del depth frame.
        depth_scale: divisor para convertir el valor crudo a metros
            (1000 para uint16/mm, 1.0 para float32/m).
        depth_percentile: percentil (0–100) usado para resumir la
            profundidad dentro del bbox. Por defecto 10 — sesgado fuerte
            hacia el primer plano. Importante: si el bbox de Gemini es
            laxo (un objeto pequeño ocupando <30% del área del bbox),
            la mediana tira hacia el FONDO (suelo/pared detrás del
            objeto) y terminamos calculando que el objeto está más
            lejos de lo que realmente está → el robot lo embiste. Con
            percentil 10, mientras el objeto ocupe al menos ~10% del
            bbox, se elige su profundidad.

    Returns:
        (x, y, z) en metros en el frame óptico de la cámara
        (z = forward, x = right, y = down), o `None` si no hay profundidad
        fiable en la zona.
    """
    if depth_image is None or depth_image.size == 0:
        return None
    if len(bbox) != 4:
        return None

    h, w = depth_image.shape[:2]
    x0, y0, x1, y1 = bbox
    u0 = max(0, int(round(x0 * w)))
    u1 = min(w, int(round(x1 * w)))
    v0 = max(0, int(round(y0 * h)))
    v1 = min(h, int(round(y1 * h)))
    if u1 <= u0 or v1 <= v0:
        return None

    region = depth_image[v0:v1, u0:u1].astype(np.float32)
    valid = region[np.isfinite(region) & (region > 0)]
    if valid.size == 0:
        return None
    z = float(np.percentile(valid, depth_percentile)) / float(depth_scale)
    if z <= 0.0 or not math.isfinite(z) or z > 20.0:
        return None

    # Dirección desde el centro del bbox (asumimos que el centro está
    # sobre el objeto). Distancia desde el percentil del bbox completo.
    u = (u0 + u1) // 2
    v = (v0 + v1) // 2
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return (x, y, z)
