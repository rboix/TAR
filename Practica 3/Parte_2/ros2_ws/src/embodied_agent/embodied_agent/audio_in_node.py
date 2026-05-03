"""
audio_in_node: Captura audio del micrófono en ventanas de 5s con Whisper.
Acumula texto hasta detectar silencio, publica en /user_speech y espera
a que brain_node termine antes de volver a escuchar.
"""
import os
import sys
import threading
import wave
import tempfile
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


WHISPER_SR = 16000  # tasa que espera Whisper
CHUNK_SIZE = 1024   # frames por bloque
WINDOW_SECS = 5     # segundos por ventana de grabación


def _resample(audio: np.ndarray, orig_sr: int, target_sr: int) -> np.ndarray:
    if orig_sr == target_sr:
        return audio
    new_len = int(len(audio) * target_sr / orig_sr)
    return np.interp(
        np.linspace(0, len(audio) - 1, new_len),
        np.arange(len(audio)),
        audio,
    ).astype(np.float32)


def _load_whisper_safe():
    os.environ['NUMBA_DISABLE_JIT'] = '1'
    os.environ.pop('COVERAGE_PROCESS_START', None)
    import whisper
    return whisper.load_model('base')


class AudioInNode(Node):
    def __init__(self):
        super().__init__('audio_in_node')
        self._pub = self.create_publisher(String, '/user_speech', 10)
        self._sub_ready = self.create_subscription(
            Bool, '/brain_ready', self._on_brain_ready, 10)
        self._whisper_model = None
        self._brain_ready = True  # al inicio está listo

        threading.Thread(target=self._load_whisper, daemon=True).start()
        threading.Thread(target=self._capture_loop, daemon=True).start()
        self.get_logger().info('audio_in_node arrancado — cargando Whisper...')

    def _on_brain_ready(self, msg: Bool):
        if msg.data:
            self._brain_ready = True
            self.get_logger().info('Brain listo — escuchando de nuevo')

    def _load_whisper(self):
        try:
            self._whisper_model = _load_whisper_safe()
            self.get_logger().info('Whisper modelo "base" cargado')
        except Exception as e:
            self.get_logger().error(f'Error cargando Whisper: {e}')

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

            while rclpy.ok():
                # Espera a que brain_node termine el ciclo anterior
                if not self._brain_ready:
                    import time
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

                audio = np.concatenate(window)
                audio_16k = _resample(audio, native_sr, WHISPER_SR)
                threading.Thread(
                    target=self._transcribe_and_accumulate,
                    args=(audio_16k,), daemon=True).start()

    # Acumulador compartido entre ventanas
    _accumulated: list[str] = []
    _accumulate_lock = threading.Lock()
    _silence_timer: threading.Timer | None = None

    def _transcribe_and_accumulate(self, audio: np.ndarray):
        if self._whisper_model is None:
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
            self.get_logger().info(f'[audio_in] publicando: "{full_text}"')
            self._brain_ready = False
            msg = String()
            msg.data = full_text
            self._pub.publish(msg)

    def _transcribe(self, audio: np.ndarray) -> tuple[str, bool]:
        tmp_path = None
        try:
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as f:
                tmp_path = f.name
            audio_int16 = (audio * 32767).astype(np.int16)
            with wave.open(tmp_path, 'wb') as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)
                wf.setframerate(WHISPER_SR)
                wf.writeframes(audio_int16.tobytes())
            result = self._whisper_model.transcribe(tmp_path, language='es')
            segs = result.get('segments', [])
            no_speech = segs[0].get('no_speech_prob', 1.0) if segs else 1.0
            text = result.get('text', '').strip()
            has_speech = bool(text) and no_speech < 0.6
            return text, has_speech
        except Exception as e:
            self.get_logger().error(f'Error transcripción: {e}')
            return '', False
        finally:
            if tmp_path:
                try:
                    os.unlink(tmp_path)
                except OSError:
                    pass


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
