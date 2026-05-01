"""
speech_node: Suscrito a /robot_speech. Sintetiza texto a audio
con ElevenLabs (o gTTS como fallback) y lo reproduce.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SpeechNode(Node):
    def __init__(self):
        super().__init__('speech_node')
        self._sub = self.create_subscription(
            String, '/robot_speech', self._on_speech, 10)
        self.get_logger().info('speech_node arrancado (stub Fase 0)')

    def _on_speech(self, msg: String):
        self.get_logger().info(f'[speech] recibido: {msg.data}')


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
