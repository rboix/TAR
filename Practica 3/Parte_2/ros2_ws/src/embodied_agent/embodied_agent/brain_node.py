"""
brain_node: Cerebro principal del agente. Suscrito a /user_speech y
sensores. Llama a Gemini, parsea JSON y despacha acciones.
"""
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String


class BrainNode(Node):
    def __init__(self):
        super().__init__('brain_node')
        self._sub_speech = self.create_subscription(
            String, '/user_speech', self._on_user_speech, 10)
        self._pub_speech = self.create_publisher(String, '/robot_speech', 10)
        self.get_logger().info('brain_node arrancado (stub Fase 0)')

    def _on_user_speech(self, msg: String):
        self.get_logger().info(f'[brain] usuario dijo: {msg.data}')

    def _publish_speech(self, text: str):
        msg = String()
        msg.data = text
        self._pub_speech.publish(msg)


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
