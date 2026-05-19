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
from embodied_agent.prompts import (
    SYSTEM_PROMPT, CONTEXT_TEMPLATE,
    PANORAMIC_USER_TEXT, INSPECT_USER_TEXT,
    INVESTIGATION_PLAN_USER_TEXT, INVESTIGATION_STEP_USER_TEXT,
    INVESTIGATION_HYPOTHESIS_USER_TEXT,
)
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

        # Frames acumulados durante una acción panorámica.
        self._panoramic_frames: list[bytes] = []
        self._panoramic_lock = threading.Lock()
        # Grados totales de la panorámica en curso (para el prompt).
        self._panoramic_degrees: float = 360.0

        # Estado de la investigación autónoma (Fase 7).
        self._investigation_active: bool = False
        self._investigation_plan: list[dict] = []
        self._investigation_step: int = 0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._sub_speech = self.create_subscription(
            String, '/user_speech', self._on_user_speech, 10)
        self._sub_image = self.create_subscription(
            Image, '/camera/image_raw', self._on_image, sensor_qos)
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
        self._pub_action = self.create_publisher(String, '/action_command', 10)
        self._pub_ready = self.create_publisher(Bool, '/brain_ready', 10)
        # Resultado del action_executor (succeeded/failed/skipped) — no
        # bloqueamos el turno esperándolo, pero lo logueamos y guardamos
        # para futura integración.
        self._sub_action_result = self.create_subscription(
            String, '/action_result', self._on_action_result, 10)

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

            should_investigate = self._maybe_update_mode(user_text)

            # Trigger de investigación autónoma: cortocircuitar Gemini y
            # disparar directamente la secuencia panoramic → plan → pasos.
            if should_investigate and not self._investigation_active:
                speech = (
                    'Entendido. Voy a investigar la escena de forma autónoma. '
                    'Comenzando con una vista panorámica completa.'
                )
                self._publish_speech(speech)
                self._memory.add_turn(
                    user=user_text, robot_said=speech, action='investigate')
                self._investigation_active = True
                self._investigation_step = 0
                self._investigation_plan = []
                self._memory.reset_investigation()
                self._memory.set_mode('autonomous')
                with self._panoramic_lock:
                    self._panoramic_frames.clear()
                self._panoramic_degrees = 360.0
                payload = {
                    'action': 'panoramic',
                    'target': None, 'image_bbox': None,
                    'distance': None, 'degrees': 360.0,
                }
                action_msg = String()
                action_msg.data = json.dumps(payload, ensure_ascii=False)
                self._pub_action.publish(action_msg)
                self.get_logger().info(
                    '[brain] investigate → panoramic 360° (disparado por keyword, '
                    'sin pasar por Gemini)')
                return

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
            self._dispatch_action(response)
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

    def _maybe_update_mode(self, user_text: str) -> bool:
        """Actualiza el modo según el texto del usuario.

        Devuelve True si se detectaron triggers de investigación autónoma
        (el caller debe iniciar la secuencia sin pasar por Gemini).
        """
        low = user_text.lower()
        if any(k in low for k in _AUTONOMOUS_TRIGGERS):
            if self._memory.mode != 'autonomous':
                self.get_logger().info('[brain] cambiando a modo AUTÓNOMO')
                self._memory.set_mode('autonomous')
            return True
        elif any(k in low for k in _GUIDED_TRIGGERS):
            if self._memory.mode != 'guided':
                self.get_logger().info('[brain] cambiando a modo GUIADO')
                self._memory.set_mode('guided')
        return False

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

    def _dispatch_action(self, response: GeminiResponse) -> None:
        """Publica el comando de acción en /action_command (JSON string).

        Wire format consumido por action_executor_node:
          {
            "action":      "none|navigate|rotate|inspect|panoramic|investigate|ask_user",
            "target":      str | None,
            "image_bbox":  [x0,y0,x1,y1] normalizado a [0,1] | None,
            "distance":    float | None,   # metros, para navigate/inspect
            "degrees":     float | None,   # grados, para rotate/panoramic
          }
        """
        # investigate → iniciar secuencia autónoma: reset + panoramic 360°.
        if response.action == 'investigate':
            self._investigation_active = True
            self._investigation_step = 0
            self._investigation_plan = []
            self._memory.reset_investigation()
            self._memory.set_mode('autonomous')
            with self._panoramic_lock:
                self._panoramic_frames.clear()
            self._panoramic_degrees = 360.0
            payload = {
                'action': 'panoramic',
                'target': None, 'image_bbox': None,
                'distance': None, 'degrees': 360.0,
            }
            msg = String()
            msg.data = json.dumps(payload, ensure_ascii=False)
            self._pub_action.publish(msg)
            self.get_logger().info(
                '[brain] investigate → dispatching panoramic 360°')
            return

        # Para panoramic: reiniciar la lista de frames acumulados y guardar grados.
        if response.action == 'panoramic':
            with self._panoramic_lock:
                self._panoramic_frames.clear()
            self._panoramic_degrees = float(
                response.action_params.degrees or 360.0)
            self.get_logger().info(
                f'[brain] panoramic iniciada: '
                f'{self._panoramic_degrees:.0f}°, frames reiniciados'
            )
        params = response.action_params
        payload = {
            'action': response.action,
            'target': params.target,
            'image_bbox': params.image_bbox,
            'distance': params.distance,
            'degrees': params.degrees,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._pub_action.publish(msg)
        self.get_logger().info(f'[brain] /action_command → {msg.data}')

    def _on_action_result(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(
                f'[brain] /action_result no es JSON válido: {msg.data}')
            return
        action = data.get('action', '')
        status = data.get('status', '')
        detail = data.get('message', '')
        self.get_logger().info(
            f'[brain] /action_result ← action={action} status={status} '
            f'msg="{detail}"'
        )

        # Captura de frame intermedio durante panorámica.
        if action == 'panoramic_step' and status == 'capture':
            with self._frame_lock:
                frame = self._latest_frame
            if frame is not None:
                with self._panoramic_lock:
                    self._panoramic_frames.append(frame)
                step = data.get('step', '?')
                total = data.get('total', '?')
                self.get_logger().info(
                    f'[brain] panoramic frame {step}/{total} capturado '
                    f'({len(frame)} bytes)'
                )
            else:
                self.get_logger().warn('[brain] panoramic_step: sin frame disponible')
            return

        # Panorámica completa → llamar a Gemini con todos los frames.
        if action == 'panoramic':
            if status == 'succeeded':
                with self._panoramic_lock:
                    frames = list(self._panoramic_frames)
                    self._panoramic_frames.clear()
                threading.Thread(
                    target=self._panoramic_followup,
                    args=(frames, self._panoramic_degrees),
                    daemon=True,
                ).start()
            else:
                self._publish_speech(
                    f'La panorámica no se ha completado bien: {detail}.')
            return

        # Inspect completado → llamar a Gemini con el frame cercano.
        if action == 'inspect':
            if status == 'succeeded':
                with self._frame_lock:
                    frame = self._latest_frame
                target = data.get('target', '')
                threading.Thread(
                    target=self._inspect_followup,
                    args=(frame, target),
                    daemon=True,
                ).start()
            else:
                speech = self._action_followup_speech(action, status, detail)
                if speech:
                    self._publish_speech(speech)
            return

        # Acciones básicas en modo investigación autónoma.
        if self._investigation_active:
            if action == 'navigate' and status == 'succeeded':
                # Gemini devolvió navigate en vez de inspect — forzamos inspect
                with self._frame_lock:
                    frame = self._latest_frame
                step = self._investigation_plan[self._investigation_step]
                target = step.get('target', '')
                threading.Thread(
                    target=self._inspect_followup,
                    args=(frame, target),
                    daemon=True,
                ).start()
                return
            if action == 'rotate' and status == 'succeeded':
                # Giramos para ver el objetivo — reintentar navegación al paso
                threading.Thread(
                    target=self._navigate_to_investigation_step,
                    args=(self._investigation_step,),
                    daemon=True,
                ).start()
                return

        # Acciones básicas (navigate, rotate): confirmar por voz.
        if action not in ('navigate', 'rotate'):
            return
        speech = self._action_followup_speech(action, status, detail)
        if speech:
            self._publish_speech(speech)

    @staticmethod
    def _action_followup_speech(action: str, status: str,
                                detail: str) -> str | None:
        if status == 'succeeded':
            if action == 'navigate':
                return 'Ya estoy aquí.'
            if action == 'rotate':
                return 'Listo, ya he girado.'
        if status == 'failed':
            if action in ('navigate', 'inspect'):
                return f'No he podido acercarme: {detail}.'
            if action == 'rotate':
                return f'No he podido completar el giro: {detail}.'
        if status == 'skipped':
            return 'No puedo hacer eso ahora, estoy ocupado con otra cosa.'
        if status == 'not_implemented_yet':
            return 'Esa acción todavía no la tengo implementada.'
        return None

    def _panoramic_followup(self, frames: list[bytes], degrees: float) -> None:
        """Llama a Gemini con todos los frames de la panorámica."""
        if not frames:
            self._publish_speech(
                'He completado la rotación, pero no he podido capturar imágenes.')
            if self._investigation_active:
                self._investigation_active = False
            self._signal_ready()
            return
        if self._processing:
            self.get_logger().warn(
                '[brain] panoramic followup: brain ocupado, abortando')
            self._publish_speech(
                'He completado la panorámica, pero estaba ocupado analizando '
                'otra cosa. Puedes pedirme que describa lo que veo.')
            if self._investigation_active:
                self._investigation_active = False
            self._signal_ready()
            return
        self._processing = True
        _chain = None  # callable a ejecutar después de liberar _processing
        try:
            if self._investigation_active:
                # ── Modo investigación autónoma: generar plan ──────────────
                self.get_logger().info(
                    f'[brain] investigación: Gemini ← {len(frames)} frames '
                    f'(generando plan)')
                context = INVESTIGATION_PLAN_USER_TEXT.format(
                    n_frames=len(frames),
                    pose=self._memory.format_pose(),
                    observations=self._memory.format_observations(),
                )
                raw = call_gemini(SYSTEM_PROMPT, context, images=frames)
                plan = raw.get('investigation_plan') or []
                speech = (raw.get('speech') or
                          'He analizado la escena. Voy a comenzar la inspección.')
                self._investigation_plan = plan
                self._memory.set_plan(plan)
                self._publish_speech(speech)
                self._memory.add_turn(
                    user='[panorámica de investigación]',
                    robot_said=speech,
                    action='panoramic',
                )
                if plan:
                    self.get_logger().info(
                        f'[brain] plan generado: {len(plan)} pasos')
                    _chain = lambda: self._navigate_to_investigation_step(0)
                else:
                    self.get_logger().warn('[brain] Gemini no devolvió plan')
                    self._publish_speech(
                        'No he podido generar un plan de inspección. '
                        'Puedes guiarme manualmente.')
                    self._investigation_active = False
            else:
                # ── Modo guiado: descripción normal de la panorámica ───────
                self.get_logger().info(
                    f'[brain] panoramic followup: Gemini ← {len(frames)} frames')
                context = PANORAMIC_USER_TEXT.format(
                    degrees=degrees,
                    n_frames=len(frames),
                    mode=self._memory.format_mode(),
                    pose=self._memory.format_pose(),
                    observations=self._memory.format_observations(),
                )
                raw = call_gemini(SYSTEM_PROMPT, context, images=frames)
                self._maybe_fix_bbox(raw, self._latest_frame_wh)
                response = self._validate_response(raw)
                self._publish_speech(response.speech)
                if response.observations:
                    self._memory.add_observations(
                        [o.model_dump() for o in response.observations])
                self._memory.add_turn(
                    user='[panorámica automática]',
                    robot_said=response.speech,
                    action='panoramic',
                )
                self._save_interaction(
                    '[panorámica automática]', response, elapsed=0.0)
        except ValidationError as e:
            self.get_logger().error(
                f'[brain] panoramic followup JSON inválido: {e.errors()}')
            self._publish_speech(
                'He completado la panorámica, pero tuve un problema al '
                'analizar las imágenes.')
            if self._investigation_active:
                self._investigation_active = False
        except Exception as e:
            self.get_logger().error(f'[brain] panoramic followup error: {e}')
            self._publish_speech(
                'He completado la panorámica. Puedes pedirme que describa '
                'lo que veo.')
            if self._investigation_active:
                self._investigation_active = False
        finally:
            self._processing = False
            if _chain is not None:
                threading.Thread(target=_chain, daemon=True).start()
            else:
                self._signal_ready()

    def _inspect_followup(self, frame: bytes | None, target: str) -> None:
        """Llama a Gemini con el frame cercano para análisis detallado."""
        if self._processing:
            self.get_logger().warn(
                '[brain] inspect followup: brain ocupado, fallback speech')
            self._publish_speech('Ya estoy aquí.')
            if self._investigation_active:
                self._investigation_active = False
                self._signal_ready()
            return
        if frame is None:
            self._publish_speech(
                'He llegado, pero no tengo imagen para analizar.')
            if self._investigation_active:
                # Continuar con el siguiente paso aunque no haya imagen
                next_idx = self._investigation_step + 1
                threading.Thread(
                    target=self._navigate_to_investigation_step,
                    args=(next_idx,), daemon=True).start()
            else:
                self._signal_ready()
            return
        self._processing = True
        _chain = None
        try:
            self.get_logger().info(
                f'[brain] inspect followup: Gemini ← frame cercano '
                f'de "{target}"')
            context = INSPECT_USER_TEXT.format(
                target=target or 'el objetivo',
                mode=self._memory.format_mode(),
                pose=self._memory.format_pose(),
                observations=self._memory.format_observations(),
            )
            raw = call_gemini(SYSTEM_PROMPT, context, image_bytes=frame)
            self._maybe_fix_bbox(raw, self._latest_frame_wh)
            response = self._validate_response(raw)
            self._publish_speech(response.speech)
            if response.observations:
                self._memory.add_observations(
                    [o.model_dump() for o in response.observations])
            self._memory.add_turn(
                user=f'[inspección automática de "{target}"]',
                robot_said=response.speech,
                action='inspect',
            )
            self._save_interaction(
                f'[inspección de "{target}"]', response, elapsed=0.0)

            if self._investigation_active:
                # Registrar observación del paso y avanzar
                self._memory.add_investigation_observation(
                    step=self._investigation_step + 1,
                    target=target,
                    observation=response.speech,
                )
                self._memory.update_plan_step(
                    self._investigation_step + 1, 'completed')
                next_idx = self._investigation_step + 1
                if next_idx >= len(self._investigation_plan):
                    _chain = self._investigation_final_hypothesis
                else:
                    _chain = lambda idx=next_idx: (
                        self._navigate_to_investigation_step(idx))
        except ValidationError as e:
            self.get_logger().error(
                f'[brain] inspect followup JSON inválido: {e.errors()}')
            self._publish_speech('Ya estoy aquí.')
            if self._investigation_active:
                next_idx = self._investigation_step + 1
                _chain = lambda idx=next_idx: (
                    self._navigate_to_investigation_step(idx))
        except Exception as e:
            self.get_logger().error(f'[brain] inspect followup error: {e}')
            self._publish_speech('Ya estoy aquí.')
            if self._investigation_active:
                next_idx = self._investigation_step + 1
                _chain = lambda idx=next_idx: (
                    self._navigate_to_investigation_step(idx))
        finally:
            self._processing = False
            if _chain is not None:
                threading.Thread(target=_chain, daemon=True).start()
            else:
                self._signal_ready()

    def _navigate_to_investigation_step(self, step_idx: int) -> None:
        """Llama a Gemini para navegar al objetivo del paso `step_idx`."""
        if not self._investigation_active:
            return
        if step_idx >= len(self._investigation_plan):
            threading.Thread(
                target=self._investigation_final_hypothesis,
                daemon=True).start()
            return

        step = self._investigation_plan[step_idx]
        self._investigation_step = step_idx
        target = step.get('target', f'objetivo {step_idx + 1}')
        reason = step.get('reason', '')
        n_total = len(self._investigation_plan)

        # Esperar a que _processing quede libre (max 30 s)
        for _ in range(300):
            if not self._processing:
                break
            time.sleep(0.1)
        else:
            self.get_logger().error(
                '[brain] timeout esperando _processing para paso investigación')
            self._investigation_active = False
            self._signal_ready()
            return

        self._processing = True
        _chain = None
        try:
            self._memory.update_plan_step(step_idx + 1, 'in_progress')
            with self._frame_lock:
                image_bytes = self._latest_frame
                frame_wh = self._latest_frame_wh

            context = INVESTIGATION_STEP_USER_TEXT.format(
                step=step_idx + 1,
                total=n_total,
                target=target,
                reason=reason,
                mode=self._memory.format_mode(),
                pose=self._memory.format_pose(),
                observations=self._memory.format_observations(),
                plan=self._memory.format_plan(),
            )
            self.get_logger().info(
                f'[brain] investigación paso {step_idx+1}/{n_total}: "{target}"')
            t0 = time.time()
            raw = call_gemini(SYSTEM_PROMPT, context, image_bytes=image_bytes)
            elapsed = time.time() - t0
            self._maybe_fix_bbox(raw, frame_wh)
            response = self._validate_response(raw)

            self._publish_speech(response.speech)
            if response.observations:
                self._memory.add_observations(
                    [o.model_dump() for o in response.observations])
            self._memory.add_turn(
                user=f'[investigación paso {step_idx+1}: {target}]',
                robot_said=response.speech,
                action=response.action,
            )
            self._save_interaction(
                f'[investigación paso {step_idx+1}]', response, elapsed)

            # Despachar acción (inspect/rotate — nunca investigate)
            params = response.action_params
            payload = {
                'action': response.action,
                'target': params.target,
                'image_bbox': params.image_bbox,
                'distance': params.distance,
                'degrees': params.degrees,
            }
            msg = String()
            msg.data = json.dumps(payload, ensure_ascii=False)
            self._pub_action.publish(msg)
            self.get_logger().info(
                f'[brain] investigación paso {step_idx+1} → /action_command '
                f'{msg.data}')

        except ValidationError as e:
            self.get_logger().error(
                f'[brain] navigate_to_step JSON inválido: {e.errors()}')
            # Saltar al siguiente paso
            next_idx = step_idx + 1
            _chain = lambda idx=next_idx: self._navigate_to_investigation_step(idx)
        except Exception as e:
            self.get_logger().error(f'[brain] navigate_to_step error: {e}')
            next_idx = step_idx + 1
            _chain = lambda idx=next_idx: self._navigate_to_investigation_step(idx)
        finally:
            self._processing = False
            if _chain is not None:
                threading.Thread(target=_chain, daemon=True).start()
            # Caso normal: esperamos que _on_action_result dispare el siguiente paso

    def _investigation_final_hypothesis(self) -> None:
        """Llamada tras completar todos los pasos del plan."""
        # Esperar _processing libre
        for _ in range(300):
            if not self._processing:
                break
            time.sleep(0.1)
        else:
            self.get_logger().error('[brain] timeout para hipótesis final')
            self._investigation_active = False
            self._signal_ready()
            return

        self._processing = True
        try:
            with self._frame_lock:
                image_bytes = self._latest_frame

            context = INVESTIGATION_HYPOTHESIS_USER_TEXT.format(
                n_steps=len(self._investigation_plan),
                all_observations=self._memory.format_observations(),
                step_observations=self._memory.format_investigation_observations(),
                pose=self._memory.format_pose(),
            )
            self.get_logger().info('[brain] investigación: generando hipótesis final')
            t0 = time.time()
            raw = call_gemini(SYSTEM_PROMPT, context, image_bytes=image_bytes)
            elapsed = time.time() - t0
            speech = (raw.get('speech') or
                      'He completado la investigación autónoma.')
            self._publish_speech(speech)
            self._memory.add_turn(
                user='[hipótesis final]',
                robot_said=speech,
                action='none',
            )
            entry = {
                'timestamp': time.strftime('%H:%M:%S'),
                'mode': self._memory.mode,
                'pose': list(self._memory.get_pose()),
                'observations_total': len(self._memory.observations),
                'user': '[hipótesis final]',
                'response': raw,
                'latency_s': round(elapsed, 2),
            }
            try:
                with INTERACTIONS_FILE.open('a', encoding='utf-8') as f:
                    f.write(json.dumps(entry, ensure_ascii=False) + '\n')
            except Exception:
                pass
            self.get_logger().info('[brain] investigación autónoma completada')
        except Exception as e:
            self.get_logger().error(f'[brain] hipótesis final error: {e}')
            self._publish_speech(
                'He completado la investigación. '
                'Puedes preguntarme mis conclusiones.')
        finally:
            self._investigation_active = False
            self._processing = False
            self._signal_ready()

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
