"""
speech_node: Suscrito a /robot_speech.
Sintetiza texto con ElevenLabs (fallback gTTS) y reproduce el audio.
"""
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from embodied_agent.tts_client import speak


class SpeechNode(Node):
    def __init__(self):
        super().__init__('speech_node')
        self._sub = self.create_subscription(
            String, '/robot_speech', self._on_speech, 10)
        self._speaking = False
        self._lock = threading.Lock()
        self.get_logger().info('speech_node arrancado')

    # Callback del subscriber. Si ya se está reproduciendo algo, descarta el nuevo mensaje.
    def _on_speech(self, msg: String):
        text = msg.data.strip()
        if not text:
            return
        with self._lock:
            if self._speaking:
                self.get_logger().warn('TTS ocupado, descartando mensaje')
                return
            self._speaking = True
        threading.Thread(target=self._play, args=(text,), daemon=True).start()

    # Llama a tts_client.speak en un hilo separado para no bloquear el nodo
    # Maneja errores y asegura que _speaking se resetee.
    # Libera el flag _speaking al finalizar, incluso si ocurre un error, para permitir futuros mensajes.
    def _play(self, text: str):
        self.get_logger().info(f'[speech] reproduciendo: "{text}"')
        try:
            speak(text)
        except Exception as e:
            self.get_logger().error(f'[speech] error TTS: {e}')
        finally:
            with self._lock:
                self._speaking = False


def main(args=None):
    rclpy.init(args=args)
    node = SpeechNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
