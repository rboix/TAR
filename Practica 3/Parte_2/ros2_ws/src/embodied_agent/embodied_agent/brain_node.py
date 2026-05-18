"""
brain_node: Suscrito a /user_speech y /camera/image_raw.
Llama a Gemini con la imagen actual y el texto del usuario, valida la
respuesta con pydantic, publica el "speech" a /robot_speech y mantiene
la memoria episódica.
"""
import io
import json
import threading
import time
from pathlib import Path

import numpy as np
from PIL import Image as PILImage
from pydantic import ValidationError
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
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


class BrainNode(Node):
    def __init__(self):
        super().__init__('brain_node')
        self._latest_frame: bytes | None = None
        self._frame_lock = threading.Lock()
        self._processing = False
        self._memory = AgentMemory()

        self._sub_speech = self.create_subscription(
            String, '/user_speech', self._on_user_speech, 10)
        self._sub_image = self.create_subscription(
            Image, '/camera/image_raw', self._on_image, 10)
        self._pub_speech = self.create_publisher(String, '/robot_speech', 10)
        self._pub_ready = self.create_publisher(Bool, '/brain_ready', 10)

        self.get_logger().info(
            'brain_node arrancado (modo detective) — esperando /user_speech')

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
            self._maybe_update_mode(user_text)

            with self._frame_lock:
                image_bytes = self._latest_frame
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

            response = self._validate_response(raw)

            self._publish_speech(response.speech)
            self._save_interaction(user_text, response, elapsed)

            self._memory.add_turn(
                user=user_text,
                robot_said=response.speech,
                action=response.action,
            )
            if response.observations:
                self._memory.add_observations(
                    [o.model_dump() for o in response.observations])

            self.get_logger().info(
                f'[brain] acción={response.action} | speech="{response.speech}"'
            )
            if response.observations:
                obs_str = ', '.join(o.label for o in response.observations)
                self.get_logger().info(f'[brain] observaciones: {obs_str}')
        except ValidationError as e:
            self.get_logger().error(
                f'[brain] JSON inválido de Gemini: {e.errors()}')
        except Exception as e:
            self.get_logger().error(f'[brain] error en ciclo: {e}')
        finally:
            self._processing = False
            self._signal_ready()

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
            history=self._memory.format_history(),
            observations=self._memory.format_observations(),
            user_text=user_text,
        )

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
