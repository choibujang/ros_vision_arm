import threading
from queue import Queue

import rclpy
from rclpy.node import Node

from ai_server_pkg.udp_server import UDPServer
from ai_server_pkg.ai_engine import AIEngine

from ros_interfaces.srv import GetDetectedObjects

class AIServerNode(Node):
    def __init__(self):
        super().__init__('ai_server_node')
        self.frame_queue = Queue(maxsize=5)

        # { device_id : { 'banana' : { 'count' : 2, 'pixels' : [(300, 200), (350, 210)] } } }
        self.detected_objects = {}
        self.lock = threading.Lock()

        self.udp_server = UDPServer(self.frame_queue)
        self.udp_server_thread = threading.Thread(target=self.udp_server.receiver)
        self.udp_server_thread.start()
        
        self.buf_clean_thread = threading.Thread(target=self.udp_server.buffer_cleaner)
        self.buf_clean_thread.start()

        self.ai_engine = AIEngine(self.frame_queue, self.detected_objects, self.lock)
        self.ai_engine_thread = threading.Thread(target=self.ai_engine.run_detection)
        self.ai_engine_thread.start()

        self.srv = self.create_service(
            GetDetectedObjects,
            'get_detected_objects',
            self.handle_request
        )

    def handle_request(self, request, response):
        device_id = request.device_id

        with self.lock:
            device_data = self.detected_objects.get(device_id)

        if not device_data:
            response.class_names = []
            response.counts = []
            response.pixel_x = []
            response.pixel_y = []
            response.pixel_class_indices = []
            return response

        class_names = []
        counts = []
        pixel_x = []
        pixel_y = []
        pixel_class_indices = []

        for idx, (class_name, data) in enumerate(device_data.items()):
            class_names.append(class_name)
            counts.append(data['count'])
            for x, y in data['pixels']:
                pixel_x.append(x)
                pixel_y.append(y)
                pixel_class_indices.append(idx)

        response.class_names = class_names
        response.counts = counts
        response.pixel_x = pixel_x
        response.pixel_y = pixel_y
        response.pixel_class_indices = pixel_class_indices
        return response

def main():
    rclpy.init()
    node = AIServerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
