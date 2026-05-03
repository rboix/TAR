"""
brain_node: Suscrito a /user_speech y /camera/image_raw.
Llama a Gemini con la imagen actual y el texto del usuario.
Parsea el JSON, guarda interacciones y publica /brain_ready cuando termina.
"""
import json
import threading
import time
from pathlib import Path

import io

import numpy as np
from PIL import Image as PILImage
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from embodied_agent.gemini_client import call_gemini
from embodied_agent.prompts import SYSTEM_PROMPT, CONTEXT_TEMPLATE


INTERACTIONS_FILE = Path('/workspace/interactions.jsonl')
MAX_HISTORY = 5


class BrainNode(Node):
    def __init__(self):
        super().__init__('brain_node')
        self._latest_frame: bytes | None = None
        self._frame_lock = threading.Lock()
        self._processing = False
        self._history: list[dict] = []

        self._sub_speech = self.create_subscription(
            String, '/user_speech', self._on_user_speech, 10)
        self._sub_image = self.create_subscription(
            Image, '/camera/image_raw', self._on_image, 10)
        self._pub_ready = self.create_publisher(Bool, '/brain_ready', 10)

        self.get_logger().info('brain_node arrancado — esperando /user_speech')

    def _on_image(self, msg: Image):
        try:
            enc = msg.encoding.lower()
            dtype = np.uint16 if '16' in enc else np.uint8
            arr = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width, -1)
            if enc in ('bgr8', 'bgra8'):
                arr = arr[:, :, ::-1]  # BGR→RGB / BGRA→RGBA
            if arr.shape[2] == 4:
                arr = arr[:, :, :3]
            buf = io.BytesIO()
            PILImage.fromarray(arr.astype(np.uint8)).save(buf, format='JPEG', quality=85)
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
            with self._frame_lock:
                image_bytes = self._latest_frame

            if image_bytes is None:
                self.get_logger().warn('[brain] sin imagen disponible, llamando solo con texto')

            context = self._build_context(user_text)
            self.get_logger().info(f'[brain] llamando a Gemini para: "{user_text}"')
            t0 = time.time()
            response = call_gemini(SYSTEM_PROMPT, context, image_bytes)
            elapsed = time.time() - t0
            self.get_logger().info(f'[brain] Gemini respondió en {elapsed:.1f}s')

            self._validate_response(response)
            self._save_interaction(user_text, response)
            self._update_history(user_text, response)

            self.get_logger().info(
                f'[brain] acción={response.get("action")}, '
                f'speech="{response.get("speech", "")}"'
            )
        except Exception as e:
            self.get_logger().error(f'[brain] error en ciclo: {e}')
        finally:
            self._processing = False
            self._signal_ready()

    def _build_context(self, user_text: str) -> str:
        if self._history:
            lines = []
            for entry in self._history[-MAX_HISTORY:]:
                ts = entry.get('timestamp', '')
                u = entry.get('user', '')
                r = entry.get('robot_said', '')
                lines.append(f'[{ts}] Usuario: {u} → Robot: {r}')
            history_str = '\n'.join(lines)
        else:
            history_str = '(sin historial previo)'

        return CONTEXT_TEMPLATE.format(
            history=history_str,
            semantic_map='(mapa semántico vacío)',
            user_text=user_text,
        )

    def _validate_response(self, resp: dict) -> None:
        for field in ('action', 'speech', 'reasoning'):
            if field not in resp:
                raise ValueError(f'Campo requerido ausente en respuesta Gemini: {field}')
        valid_actions = {
            'none', 'navigate', 'rotate', 'search',
            'follow_person', 'go_home', 'ask_user', 'report',
        }
        if resp.get('action') not in valid_actions:
            raise ValueError(f'Acción inválida: {resp.get("action")}')

    def _save_interaction(self, user_text: str, response: dict) -> None:
        entry = {
            'timestamp': time.strftime('%H:%M:%S'),
            'user': user_text,
            'response': response,
        }
        try:
            with INTERACTIONS_FILE.open('a', encoding='utf-8') as f:
                f.write(json.dumps(entry, ensure_ascii=False) + '\n')
        except Exception as e:
            self.get_logger().error(f'[brain] no se pudo guardar interacción: {e}')

    def _update_history(self, user_text: str, response: dict) -> None:
        self._history.append({
            'timestamp': time.strftime('%H:%M:%S'),
            'user': user_text,
            'robot_said': response.get('speech', ''),
            'action': response.get('action', 'none'),
        })

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
