"""
audio_in_node: Captura audio del micrófono en ventanas de 5s, transcribe
con Whisper (API OpenAI por defecto, fallback local) y publica en
/user_speech. Acumula texto entre ventanas hasta detectar un silencio
de 1s y espera a que brain_node termine antes de volver a escuchar.

El backend STT se decide en `stt_client.initialize()` (mira la docstring
de ese módulo para variables de entorno relevantes).
"""
import os
import threading

import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

from embodied_agent import stt_client


STT_SR = 16000     # tasa de muestreo que se envía al backend STT
CHUNK_SIZE = 1024  # frames por bloque de lectura del mic
WINDOW_SECS = 5    # segundos por ventana de grabación

# Umbral de RMS por debajo del cual no llamamos al backend STT. En
# float32 [-1,1] el ruido de fondo de un mic típico está en 0.001–0.003;
# el habla suele estar por encima de 0.02. Con OpenAI (sin no_speech_prob)
# el gate es la primera línea de defensa, así que se sube un poco.
# Configurable via AUDIO_RMS_GATE=... en .env.
_RMS_GATE = float(os.environ.get('AUDIO_RMS_GATE', '0.018'))

# Número mínimo de palabras y chars (sin puntuación) para publicar.
# Con OpenAI whisper-1 no hay no_speech_prob, así que estos filtros son
# la segunda línea contra alucinaciones de 1-2 palabras sobre ruido.
_MIN_WORDS = int(os.environ.get('AUDIO_MIN_WORDS', '2'))
_MIN_CHARS = int(os.environ.get('AUDIO_MIN_CHARS', '6'))

# Alucinaciones de Whisper/gpt-4o sobre silencio o ruido ambiental.
# Vienen del dataset de YouTube (subtítulos) o son frases cortas que
# el modelo "completa" sobre ruido de fondo.
_WHISPER_HALLUCINATIONS = (
    # Dataset YouTube (Whisper local y API)
    'amara.org',
    'subtítulos realizados por',
    'subtitulado por',
    'subtítulos por',
    'más información',
    'mas informacion',
    'gracias por ver',
    'gracias por su atención',
    'suscríbete',
    'subscribe',
    'thanks for watching',
    'thank you for watching',
    # Alucinaciones frecuentes de whisper-1/gpt-4o sobre ruido de fondo
    'la veracidad',
    'veracidad',
    'la gente',
    'las personas',
    'la vida',
    'por favor',
    'muchas gracias',
    'buenas tardes',
    'buenas noches',
    'buenos días',
    'hasta luego',
    'de asistir',
    'que son',
    'la caña',
)


def _looks_like_hallucination(text: str) -> bool:
    if not text:
        return False
    low = text.lower()
    if any(p in low for p in _WHISPER_HALLUCINATIONS):
        # "más información" y similares son especialmente típicas en silencio.
        # Para no cargarnos casos legítimos, aplicamos heurística extra:
        # si viene con una URL, casi seguro es filler del dataset.
        if ('más información' in low or 'mas informacion' in low) and (
            'www.' in low or 'http://' in low or 'https://' in low
        ):
            return True
        # Para el resto de frases, match directo es suficiente.
        if ('más información' in low or 'mas informacion' in low):
            # "Más información" a secas también suele ser alucinación.
            return True
        return True

    # Heurística URL: si lo único “informativo” es una URL (o "www..."),
    # y el texto es corto, suele ser alucinación en silencio.
    # No filtramos URLs largas con más contexto para no romper casos reales.
    if ('www.' in low or 'http://' in low or 'https://' in low) and len(low) <= 60:
        return True
    return False


def _resample(audio: np.ndarray, orig_sr: int, target_sr: int) -> np.ndarray:
    if orig_sr == target_sr:
        return audio
    new_len = int(len(audio) * target_sr / orig_sr)
    return np.interp(
        np.linspace(0, len(audio) - 1, new_len),
        np.arange(len(audio)),
        audio,
    ).astype(np.float32)


class AudioInNode(Node):
    # Tras un TTS, ignoramos N segundos extra para descartar el eco residual
    # que quedó en el buffer del stream antes de marcar el robot como callado.
    POST_SPEECH_TAIL_S = float(os.environ.get('AUDIO_POST_SPEECH_TAIL_S', '4.0'))
    # Ventana temporal (s) en la que, si transcribimos algo muy parecido a lo
    # último que el robot ha dicho, lo tratamos como eco y lo descartamos.
    ROBOT_ECHO_WINDOW_S = float(os.environ.get('AUDIO_ROBOT_ECHO_WINDOW_S', '12.0'))
    # Similaridad mínima (0..1) para considerar que es eco del TTS.
    ROBOT_ECHO_MIN_SIM = float(os.environ.get('AUDIO_ROBOT_ECHO_MIN_SIM', '0.86'))

    def __init__(self):
        super().__init__('audio_in_node')
        self._pub = self.create_publisher(String, '/user_speech', 10)
        self._sub_ready = self.create_subscription(
            Bool, '/brain_ready', self._on_brain_ready, 10)
        self._sub_speaking = self.create_subscription(
            Bool, '/robot_speaking', self._on_robot_speaking, 10)
        # Para filtrar eco: texto exacto que intentamos decir (antes de STT)
        self._sub_robot_speech = self.create_subscription(
            String, '/robot_speech', self._on_robot_speech, 10)
        self._stt_ready = False
        self._brain_ready = True  # al inicio está listo
        self._robot_speaking = False
        self._mute_until = 0.0  # timestamp hasta el cual ignorar el mic
        self._last_robot_speech = ''
        self._last_robot_speech_t = 0.0

        threading.Thread(target=self._load_stt, daemon=True).start()
        threading.Thread(target=self._capture_loop, daemon=True).start()
        self.get_logger().info('audio_in_node arrancado — cargando STT...')

    def _on_brain_ready(self, msg: Bool):
        if msg.data:
            self._brain_ready = True
            # No es que “escuche” literalmente si el robot sigue hablando;
            # el bucle de captura seguirá silenciado por /robot_speaking.
            self.get_logger().info('Brain listo — preparado para escuchar')

    def _on_robot_speech(self, msg: String):
        import time as _t
        text = (msg.data or '').strip()
        if not text:
            return
        self._last_robot_speech = text
        self._last_robot_speech_t = _t.monotonic()

    def _on_robot_speaking(self, msg: Bool):
        import time as _t
        if msg.data:
            self._robot_speaking = True
            # Tira cualquier transcripción parcial: si vino de mientras el
            # robot arrancaba a hablar, casi seguro es eco.
            with self._accumulate_lock:
                self._accumulated.clear()
                if self._silence_timer:
                    self._silence_timer.cancel()
                    self._silence_timer = None
            self.get_logger().info('Robot hablando — mic en silencio')
        else:
            self._robot_speaking = False
            self._mute_until = _t.monotonic() + self.POST_SPEECH_TAIL_S

    def _load_stt(self):
        try:
            backend = stt_client.initialize(logger=self.get_logger())
            self._stt_ready = True
            self.get_logger().info(f'STT listo (backend={backend})')
        except Exception as e:
            self.get_logger().error(f'Error inicializando STT: {e}')

    def _capture_loop(self):
        try:
            import sounddevice as sd
        except Exception as e:
            self.get_logger().error(f'sounddevice no disponible: {e}')
            return

        env_device = os.environ.get('AUDIO_INPUT_DEVICE', None)
        if env_device is not None:
            try:
                device_id = int(env_device)
            except ValueError:
                device_id = env_device
        else:
            device_id = None
            all_devices = sd.query_devices()
            input_devices = [(i, d) for i, d in enumerate(all_devices)
                             if d['max_input_channels'] > 0]
            dev_names = ', '.join(f'{i}:{d["name"]}' for i, d in input_devices)
            self.get_logger().info(f'Dispositivos de entrada: {dev_names}')
            for keyword in ('pulse', 'dmic', 'acp'):
                for i, d in input_devices:
                    if keyword in d['name'].lower():
                        device_id = i
                        break
                if device_id is not None:
                    break
            if device_id is None:
                for i, d in input_devices:
                    if 'default' not in d['name'].lower():
                        device_id = i
                        break

        try:
            device_info = sd.query_devices(device_id, kind='input')
            native_sr = int(device_info['default_samplerate'])
            self.get_logger().info(
                f'Dispositivo: {device_info["name"]} (id={device_id}) @ {native_sr} Hz')
        except Exception as e:
            self.get_logger().error(f'No se puede abrir el micrófono: {e}')
            return

        window_frames = int(WINDOW_SECS * native_sr / CHUNK_SIZE)
        self.get_logger().info(f'Escuchando (ventanas de {WINDOW_SECS}s)...')

        with sd.InputStream(device=device_id, samplerate=native_sr, channels=1,
                            dtype='float32', blocksize=CHUNK_SIZE) as stream:
            for _ in range(10):
                stream.read(CHUNK_SIZE)

            import time
            while rclpy.ok():
                # Espera a que brain_node termine el ciclo anterior
                if not self._brain_ready:
                    time.sleep(0.1)
                    continue

                # Graba una ventana
                window = []
                for _ in range(window_frames):
                    if not self._brain_ready:
                        break
                    block, _ = stream.read(CHUNK_SIZE)
                    window.append(block.flatten())

                if not window:
                    continue

                # Si el robot está hablando, o todavía estamos en el tail
                # post-TTS, tira la ventana entera. Es eco, no usuario.
                if self._robot_speaking or time.monotonic() < self._mute_until:
                    continue

                audio = np.concatenate(window)
                audio_16k = _resample(audio, native_sr, STT_SR)
                threading.Thread(
                    target=self._transcribe_and_accumulate,
                    args=(audio_16k,), daemon=True).start()

    # Acumulador compartido entre ventanas
    _accumulated: list[str] = []
    _accumulate_lock = threading.Lock()
    _silence_timer: threading.Timer | None = None

    def _transcribe_and_accumulate(self, audio: np.ndarray):
        if not self._stt_ready:
            return

        text, has_speech = self._transcribe(audio)

        with self._accumulate_lock:
            if has_speech and text:
                self._accumulated.append(text)
                self.get_logger().info(f'[audio_in] parcial: "{text}"')
                # Resetea el timer de silencio
                if self._silence_timer:
                    self._silence_timer.cancel()
                self._silence_timer = threading.Timer(
                    1.0, self._flush_accumulated)
                self._silence_timer.start()
            elif self._accumulated:
                # Ventana de silencio tras haber acumulado texto → publicar
                if self._silence_timer:
                    self._silence_timer.cancel()
                self._flush_accumulated()

    def _flush_accumulated(self):
        with self._accumulate_lock:
            if not self._accumulated:
                return
            full_text = ' '.join(self._accumulated).strip()
            self._accumulated.clear()
            self._silence_timer = None

        if full_text:
            # Filtro anti-eco: si es (casi) lo mismo que el robot acaba de
            # decir, lo descartamos. Esto cubre casos donde /robot_speaking
            # se libera un poco antes que el audio real (buffer/pipeline).
            if self._is_robot_echo(full_text):
                self.get_logger().info(
                    f'[audio_in] descartado eco del robot: "{full_text}"')
                return
            self.get_logger().info(f'[audio_in] publicando: "{full_text}"')
            self._brain_ready = False
            msg = String()
            msg.data = full_text
            self._pub.publish(msg)

    def _is_robot_echo(self, text: str) -> bool:
        import time as _t
        if not text or not self._last_robot_speech:
            return False
        if (_t.monotonic() - self._last_robot_speech_t) > self.ROBOT_ECHO_WINDOW_S:
            return False

        def _norm(s: str) -> str:
            import re
            s = s.lower().strip()
            s = re.sub(r'[^a-záéíóúñü0-9\\s]', ' ', s)
            s = re.sub(r'\\s+', ' ', s).strip()
            return s

        a = _norm(text)
        b = _norm(self._last_robot_speech)
        if not a or not b:
            return False
        # Si el texto es muy corto, evita falsos positivos: exige igualdad.
        if len(a) < 12 or len(b) < 12:
            return a == b

        from difflib import SequenceMatcher
        sim = SequenceMatcher(None, a, b).ratio()
        return sim >= self.ROBOT_ECHO_MIN_SIM

    def _transcribe(self, audio: np.ndarray) -> tuple[str, bool]:
        # Cortocircuito por RMS: si la ventana es prácticamente muda, no
        # gastamos llamada a la API ni cargamos al backend local. Como
        # bonus, evita las alucinaciones más burdas de Whisper sobre
        # silencio (que ya no llegarían ni a la API).
        rms = float(np.sqrt(np.mean(np.square(audio.astype(np.float32)))))
        if not np.isfinite(rms) or rms < _RMS_GATE:
            return '', False

        try:
            text, no_speech = stt_client.transcribe(audio, sample_rate=STT_SR)
        except Exception as e:
            self.get_logger().error(f'Error transcripción: {e}')
            return '', False

        if _looks_like_hallucination(text):
            self.get_logger().debug(
                f'[audio_in] alucinación filtrada: "{text}"')
            return '', False

        # Filtro de longitud: descarta fragmentos muy cortos que son
        # alucinaciones habituales de whisper-1 sobre ruido ambiental.
        import re as _re
        clean = _re.sub(r'[^\w\s]', '', text, flags=_re.UNICODE).strip()
        words = clean.split()
        if len(words) < _MIN_WORDS or len(clean) < _MIN_CHARS:
            self.get_logger().debug(
                f'[audio_in] descartado por longitud ({len(words)} palabras, '
                f'{len(clean)} chars): "{text}"')
            return '', False

        # Con OpenAI whisper-1 no_speech es siempre 0.0 — confiamos en
        # el RMS gate + filtros anteriores para decidir si hay habla real.
        if stt_client.active_backend() == 'local':
            has_speech = bool(text) and no_speech < 0.6
        else:
            has_speech = bool(text)
        return text, has_speech


def main(args=None):
    rclpy.init(args=args)
    node = AudioInNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
