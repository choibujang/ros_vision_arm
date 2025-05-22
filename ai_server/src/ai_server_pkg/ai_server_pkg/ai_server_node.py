import threading
from queue import Queue

import rclpy
from rclpy.node import Node
from ros_interfaces.srv import ProductDetection
from ros_interfaces.srv import StartUDPServer

from ai_server_pkg.udp_server import UDPServer

class AiServerNode(Node):
    def __init__(self):
        super().__init__('ai_server_node')
        self.queue = Queue()

        self.udp_server = UDPServer(self.queue)
        self.udp_server_thread = threading.Thread(target=self.udp_server.receiver)
        self.buf_cleaner_thread = threading.Thread(target=self.udp_server.buffer_cleaner)
        self.udp_server_thread.start()
        self.buf_cleaner_thread.start()


def main():
    rclpy.init()
    node = AiServerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
