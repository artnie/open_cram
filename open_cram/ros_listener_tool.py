"""
title: ROS listener example
author: artnie
author_url: https://github.com/artnie/open_cram
funding_url: https://github.com/artnie/open_cram
version: 0.1.0
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Tools:
    def __init__(self):
        # If set to true it will prevent default RAG pipeline
        self.citation = True
        pass

    def listen_to_chatter(self) -> str:
        # run the talker node in the same network
        # ros2 run demo_nodes_py talker
        class OneShotListener(Node):
            def __init__(self):
                super().__init__("one_shot_listener")
                self.latest_msg = None
                self.subscription = self.create_subscription(
                    String, "chatter", self.listener_callback, 10
                )

            def listener_callback(self, msg):
                self.latest_msg = msg.data
                self.get_logger().info(f"Received: {self.latest_msg}")

                # Stop spinning once the message is received
                rclpy.shutdown()

        rclpy.init()
        node = OneShotListener()

        # Spin until shutdown is triggered in the callback
        try:
            rclpy.spin(node)
        except SystemExit:
            pass

        # After shutdown, print the final value
        if node.latest_msg is not None:
            print(f"Latest message: {node.latest_msg}")
        else:
            print("No message received.")

        node.destroy_node()
        return str(node.latest_msg)
