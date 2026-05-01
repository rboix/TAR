"""
audio_in_node: Captura audio del micrófono, detecta voz (VAD),
transcribe con Whisper y publica el texto en /user_speech.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class AudioInNode(Node):
    def __init__(self):
        super().__init__('audio_in_node')
        self._pub = self.create_publisher(String, '/user_speech', 10)
        self.get_logger().info('audio_in_node arrancado (stub Fase 0)')

    def publish_text(self, text: str):
        msg = String()
        msg.data = text
        self._pub.publish(msg)
        self.get_logger().info(f'[audio_in] publicado: {text}')


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
