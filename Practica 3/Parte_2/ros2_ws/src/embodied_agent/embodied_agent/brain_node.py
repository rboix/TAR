"""
brain_node: Suscrito a /user_speech y /camera/image_raw.
Llama a Gemini con la imagen actual y el texto del usuario, valida la
respuesta con pydantic, publica el "speech" a /robot_speech y mantiene
la memoria episódica.
"""
import io
import json
import math
import threading
import time
from pathlib import Path

import numpy as np
from PIL import Image as PILImage
from pydantic import ValidationError
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from embodied_agent.gemini_client import call_gemini
from embodied_agent.memory import AgentMemory
from embodied_agent.prompts import SYSTEM_PROMPT, CONTEXT_TEMPLATE
from embodied_agent.schemas import GeminiResponse


INTERACTIONS_FILE = Path('/workspace/interactions.jsonl')

# Palabras clave que cambian el modo (heurística simple para fase 3;
# en fase 7 se gestiona desde el propio action="investigate").
_AUTONOMOUS_TRIGGERS = ('investiga', 'investigar', 'investigación')
_GUIDED_TRIGGERS = ('modo guiado', 'para de investigar', 'detente')

# Comandos para borrar la memoria episódica (útiles para testear la fase 4
# y para empezar una investigación limpia entre escenas).
_RESET_TRIGGERS = (
    'olvida todo', 'olvídalo todo', 'olvida lo que has visto',
    'empieza de nuevo', 'empezamos de nuevo', 'nueva investigación',
    'borra la memoria', 'resetea la memoria', 'reinicia la investigación',
)


class BrainNode(Node):
    def __init__(self):
        super().__init__('brain_node')
        self._latest_frame: bytes | None = None
        self._latest_frame_wh: tuple[int, int] | None = None  # (width, height)
        self._frame_lock = threading.Lock()
        self._processing = False
        self._memory = AgentMemory()

        self._sub_speech = self.create_subscription(
            String, '/user_speech', self._on_user_speech, 10)
        self._sub_image = self.create_subscription(
            Image, '/camera/image_raw', self._on_image, 10)
        # /odom puede venir con QoS BEST_EFFORT o RELIABLE según el driver.
        # Usamos BEST_EFFORT para ser compatibles con la mayoría de fuentes
        # (Gazebo diff_drive, Create 3) — el pose sólo se usa para anotar
        # turnos, no es crítico perder algún sample.
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._sub_odom = self.create_subscription(
            Odometry, '/odom', self._on_odom, odom_qos)
        self._pub_speech = self.create_publisher(String, '/robot_speech', 10)
        self._pub_ready = self.create_publisher(Bool, '/brain_ready', 10)

        self.get_logger().info(
            'brain_node arrancado (modo detective, memoria episódica activa)'
            ' — esperando /user_speech')

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        # yaw a partir del cuaternión (z, w son los relevantes en 2D).
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)
        self._memory.set_pose((p.x, p.y, theta))

    def _on_image(self, msg: Image):
        try:
            enc = msg.encoding.lower()
            dtype = np.uint16 if '16' in enc else np.uint8
            arr = np.frombuffer(msg.data, dtype=dtype).reshape(
                msg.height, msg.width, -1)
            if enc in ('bgr8', 'bgra8'):
                arr = arr[:, :, ::-1]
            if arr.shape[2] == 4:
                arr = arr[:, :, :3]
            buf = io.BytesIO()
            PILImage.fromarray(arr.astype(np.uint8)).save(
                buf, format='JPEG', quality=85)
            with self._frame_lock:
                self._latest_frame = buf.getvalue()
                self._latest_frame_wh = (int(msg.width), int(msg.height))
        except Exception as e:
            self.get_logger().warn(f'[brain] error procesando imagen: {e}')

    def _on_user_speech(self, msg: String):
        text = msg.data.strip()
        if not text:
            self._signal_ready()
            return
        if self._processing:
            self.get_logger().warn('[brain] ya procesando, descartando mensaje')
            self._signal_ready()
            return
        self._processing = True
        threading.Thread(target=self._process, args=(text,), daemon=True).start()

    def _process(self, user_text: str):
        try:
            if self._maybe_reset_memory(user_text):
                # El reset ya emite voz y guarda el turno por su cuenta.
                return

            self._maybe_update_mode(user_text)

            with self._frame_lock:
                image_bytes = self._latest_frame
                frame_wh = self._latest_frame_wh
            if image_bytes is None:
                self.get_logger().warn(
                    '[brain] sin imagen disponible, llamando solo con texto')

            context = self._build_context(user_text)
            self.get_logger().info(
                f'[brain] modo={self._memory.mode} | Gemini ← "{user_text}"')
            t0 = time.time()
            raw = call_gemini(SYSTEM_PROMPT, context, image_bytes)
            elapsed = time.time() - t0
            self.get_logger().info(f'[brain] Gemini respondió en {elapsed:.1f}s')

            self._maybe_fix_bbox(raw, frame_wh)
            response = self._validate_response(raw)

            self._publish_speech(response.speech)
            self._save_interaction(user_text, response, elapsed)

            self._memory.add_turn(
                user=user_text,
                robot_said=response.speech,
                action=response.action,
            )
            new_obs = 0
            if response.observations:
                new_obs = self._memory.add_observations(
                    [o.model_dump() for o in response.observations])

            self.get_logger().info(
                f'[brain] acción={response.action} | speech="{response.speech}"'
            )
            if response.observations:
                obs_str = ', '.join(o.label for o in response.observations)
                self.get_logger().info(
                    f'[brain] observaciones: {obs_str} '
                    f'(+{new_obs} nuevas, total={len(self._memory.observations)})'
                )
        except ValidationError as e:
            self.get_logger().error(
                f'[brain] JSON inválido de Gemini: {e.errors()}')
        except Exception as e:
            self.get_logger().error(f'[brain] error en ciclo: {e}')
        finally:
            self._processing = False
            self._signal_ready()

    def _maybe_reset_memory(self, user_text: str) -> bool:
        """Si el usuario pide olvidar/resetear, limpia memoria y responde.

        Devuelve True si se ha consumido el turno (no llamamos a Gemini).
        """
        low = user_text.lower()
        if not any(k in low for k in _RESET_TRIGGERS):
            return False
        n_obs = len(self._memory.observations)
        n_turns = len(self._memory.conversation_history)
        self._memory.reset_all()
        speech = (
            'De acuerdo, borro lo que tenía en memoria y empezamos una '
            'nueva investigación.'
        )
        self.get_logger().info(
            f'[brain] RESET memoria episódica '
            f'(borradas {n_obs} pistas y {n_turns} turnos)')
        self._publish_speech(speech)
        # Guardamos el propio turno de reset para que quede traza.
        self._memory.add_turn(
            user=user_text, robot_said=speech, action='none',
        )
        return True

    def _maybe_update_mode(self, user_text: str) -> None:
        low = user_text.lower()
        if any(k in low for k in _AUTONOMOUS_TRIGGERS):
            if self._memory.mode != 'autonomous':
                self.get_logger().info('[brain] cambiando a modo AUTÓNOMO')
                self._memory.set_mode('autonomous')
        elif any(k in low for k in _GUIDED_TRIGGERS):
            if self._memory.mode != 'guided':
                self.get_logger().info('[brain] cambiando a modo GUIADO')
                self._memory.set_mode('guided')

    def _build_context(self, user_text: str) -> str:
        return CONTEXT_TEMPLATE.format(
            mode=self._memory.format_mode(),
            pose=self._memory.format_pose(),
            history=self._memory.format_history(),
            observations=self._memory.format_observations(),
            plan=self._memory.format_plan(),
            investigation_observations=(
                self._memory.format_investigation_observations()),
            user_text=user_text,
        )

    def _maybe_fix_bbox(self, raw: dict,
                        frame_wh: tuple[int, int] | None) -> None:
        """Normaliza image_bbox si Gemini lo devuelve fuera de [0,1].

        Gemini 2.5 Flash mezcla a veces escalas: puede devolver píxeles
        ([0, 0, 640, 480]), la escala 0–1000 que usan otros modelos suyos,
        o incluso un híbrido (X normalizado, Y en píxeles). En lugar de
        rechazar el turno, lo intentamos reparar antes de validar.
        """
        if not isinstance(raw, dict):
            return
        params = raw.get('action_params')
        if not isinstance(params, dict):
            return
        bbox = params.get('image_bbox')
        if bbox is None or not isinstance(bbox, list) or len(bbox) != 4:
            return
        try:
            vals = [float(v) for v in bbox]
        except (TypeError, ValueError):
            return
        if all(0.0 <= v <= 1.0 for v in vals):
            return  # ya está normalizado

        if frame_wh is not None:
            w, h = frame_wh
            # Caso píxeles: dividir X/W y Y/H independientemente. Esto
            # también arregla el caso híbrido (X normalizada, Y en píxeles)
            # porque dividir 0.6 / 640 daría ~0.001, así que sólo
            # dividimos los componentes que exceden de 1.
            fixed = []
            for i, v in enumerate(vals):
                if 0.0 <= v <= 1.0:
                    fixed.append(v)
                else:
                    denom = w if i % 2 == 0 else h
                    fixed.append(v / denom if denom > 0 else v)
        else:
            # Sin dimensiones (frame None) probamos la escala 0–1000 de Gemini.
            fixed = [v / 1000.0 for v in vals]

        # Recortar a [0,1] por si quedó algo justo fuera por redondeo.
        fixed = [max(0.0, min(1.0, v)) for v in fixed]
        # Asegurar orden x0<x1, y0<y1; si no, lo dejamos como estaba y
        # que pydantic lo rechace (es un bbox genuinamente malformado).
        if fixed[2] > fixed[0] and fixed[3] > fixed[1]:
            self.get_logger().warn(
                f'[brain] image_bbox fuera de [0,1]: {bbox} → normalizado '
                f'a {[round(v, 3) for v in fixed]}')
            params['image_bbox'] = fixed

    def _validate_response(self, raw: dict) -> GeminiResponse:
        return GeminiResponse.model_validate(raw)

    def _publish_speech(self, speech: str) -> None:
        msg = String()
        msg.data = speech
        self._pub_speech.publish(msg)

    def _save_interaction(self, user_text: str, response: GeminiResponse,
                          elapsed: float) -> None:
        entry = {
            'timestamp': time.strftime('%H:%M:%S'),
            'mode': self._memory.mode,
            'pose': list(self._memory.get_pose()),
            'observations_total': len(self._memory.observations),
            'user': user_text,
            'response': response.model_dump(),
            'latency_s': round(elapsed, 2),
        }
        try:
            with INTERACTIONS_FILE.open('a', encoding='utf-8') as f:
                f.write(json.dumps(entry, ensure_ascii=False) + '\n')
        except Exception as e:
            self.get_logger().error(
                f'[brain] no se pudo guardar interacción: {e}')

    def _signal_ready(self):
        msg = Bool()
        msg.data = True
        self._pub_ready.publish(msg)
        self.get_logger().info('[brain] /brain_ready publicado')


def main(args=None):
    rclpy.init(args=args)
    node = BrainNode()
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
