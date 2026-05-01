"""
action_executor_node: Ejecuta las acciones físicas del robot.
Soporta: navigate, rotate, search, follow_person, go_home, report, none.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ActionExecutorNode(Node):
    def __init__(self):
        super().__init__('action_executor_node')
        self._sub = self.create_subscription(
            String, '/action_command', self._on_action, 10)
        self._pub_result = self.create_publisher(String, '/action_result', 10)
        self.get_logger().info('action_executor_node arrancado (stub Fase 0)')

    def _on_action(self, msg: String):
        self.get_logger().info(f'[action_executor] acción recibida: {msg.data}')

    def _publish_result(self, result: str):
        msg = String()
        msg.data = result
        self._pub_result.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ActionExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
