import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .clients import OpenAIClient, OpenWebUIClient

class OpenAIBridgeNode(Node):
    def __init__(self):
        super().__init__('open_cram')

        self.declare_parameter('openai_model', 'gpt-4o-mini')
        self.declare_parameter('openwebui_url', 'http://192.168.200.10:3000')
        self.declare_parameter('timeout_s', 30.0)

        model = self.get_parameter('openai_model').get_parameter_value().string_value
        ow_url = self.get_parameter('openwebui_url').get_parameter_value().string_value
        timeout = self.get_parameter('timeout_s').get_parameter_value().double_value

        self.openai = OpenAIClient(model=model, timeout_s=timeout)
        self.openwebui = OpenWebUIClient(base_url=ow_url, timeout_s=timeout)

        # Example ROS I/O
        self.sub = self.create_subscription(String, 'open_cram/in', self.on_in, 10)
        self.pub = self.create_publisher(String, 'open_cram/out', 10)

        self.get_logger().info('open_cram node started.')

    def on_in(self, msg: String):
        prompt = msg.data
        try:
            ai_answer = self.openai.chat(prompt)
            ow_answer = self.openwebui.chat(prompt)
            merged = f"[openai] {ai_answer}\n[open-webui] {ow_answer}"
            self.pub.publish(String(data=merged))
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

def main():
    rclpy.init()
    node = OpenAIBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()