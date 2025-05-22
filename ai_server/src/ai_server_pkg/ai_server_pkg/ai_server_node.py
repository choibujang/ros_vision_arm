import threading
from queue import Queue

import rclpy
from rclpy.node import Node
from ros_interfaces.srv import ProductDetection
from ros_interfaces.srv import StartUDPServer

from ai_server_pkg.udp_receiver import UDPReceiver

class AiServiceNode(Node):
    def __init__(self):
        super().__init__('ai_service_node')
        self.queue = Queue()

        self.udp_receiver = None
        self.udp_thread = None

        self.create_service(StartUDPServer, 'start_udp_srv', self.handle_start_udp)
        self.create_service(ProductDetection, 'product_detect_srv', self.handle_product_detect)

    def handle_start_udp(self, request, response):
        self.udp_receiver = UDPReceiver(self.queue, request.port)
        self.udp_thread = threading.Thread(target=self.udp_receiver.start)
        self.udp_thread.start()

        response.success = True
        return response
    
    def handle_product_detect(self, request, response):
        response.product_names = "apple"
        response.width = 100
        response.height = 100
        response.cx = 50
        response.cy = 50

        return response

def main():
    rclpy.init()
    node = AiServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
