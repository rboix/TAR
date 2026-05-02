"""
audio_in_node: Captura audio del micrófono con VAD simple (energía RMS),
transcribe con Whisper y publica el texto en /user_speech.
"""
import os
import sys
import threading
import wave
import tempfile
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


WHISPER_SR = 16000  # tasa que espera Whisper
CHUNK_SIZE = 1024   # frames por bloque


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
    # Desactiva el JIT de numba para evitar el conflicto con coverage de ROS 2
    os.environ['NUMBA_DISABLE_JIT'] = '1'
    os.environ.pop('COVERAGE_PROCESS_START', None)
    import whisper
    return whisper.load_model('base')


class AudioInNode(Node):
    def __init__(self):
        super().__init__('audio_in_node')
        self._pub = self.create_publisher(String, '/user_speech', 10)
        self._whisper_model = None

        threading.Thread(target=self._load_whisper, daemon=True).start()
        threading.Thread(target=self._capture_loop, daemon=True).start()
        self.get_logger().info('audio_in_node arrancado — cargando Whisper...')


    # Carfa el modelo base de Whisper en un hilo separado para no bloquear el nodo
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

        # Selecciona el dispositivo de entrada: env var > DMIC/acp > primer input real
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
            self.get_logger().info(f'Dispositivos de entrada disponibles: {dev_names}')
            # Prioridad: pulse (con noise cancellation) > DMIC/acp > primer input real
            for keyword in ('pulse', 'dmic', 'acp'):
                for i, d in input_devices:
                    if keyword in d['name'].lower():
                        device_id = i
                        break
                if device_id is not None:
                    break
            # Fallback: primer dispositivo de entrada que no sea el default
            if device_id is None:
                for i, d in input_devices:
                    if 'default' not in d['name'].lower():
                        device_id = i
                        break

        try:
            device_info = sd.query_devices(device_id, kind='input')
            native_sr = int(device_info['default_samplerate'])
            self.get_logger().info(
                f'Dispositivo de audio: {device_info["name"]} (id={device_id}) @ {native_sr} Hz')
        except Exception as e:
            self.get_logger().error(f'No se puede abrir el micrófono: {e}')
            return

        # Graba ventanas fijas de 10s y deja que Whisper filtre el ruido
        window_frames = int(10.0 * native_sr / CHUNK_SIZE)
        self.get_logger().info('Escuchando micrófono (ventanas de 10s)...')

        with sd.InputStream(device=device_id, samplerate=native_sr, channels=1,
                            dtype='float32', blocksize=CHUNK_SIZE) as stream:
            for _ in range(10):  # descarta bloques de inicialización
                stream.read(CHUNK_SIZE)

            while rclpy.ok():
                window = []
                for _ in range(window_frames):
                    block, _ = stream.read(CHUNK_SIZE)
                    window.append(block.flatten())
                audio = np.concatenate(window)
                audio_16k = _resample(audio, native_sr, WHISPER_SR)
                threading.Thread(
                    target=self._transcribe_and_publish,
                    args=(audio_16k,), daemon=True).start()


    def _transcribe_and_publish(self, audio: np.ndarray):
        if self._whisper_model is None:
            self.get_logger().warn('Whisper aún no cargado, descartando audio')
            return

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
            no_speech = result.get('segments', [{}])[0].get('no_speech_prob', 0.0) \
                if result.get('segments') else 1.0
            text = result.get('text', '').strip()
            if text and no_speech < 0.6:
                self.get_logger().info(f'[audio_in] transcrito: "{text}"')
                msg = String()
                msg.data = text
                self._pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error en transcripción: {e}')
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
