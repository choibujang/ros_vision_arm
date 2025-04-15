from rclpy.node import Node
from ai_server_interfaces.srv import ProductDetection
from ai_server_interfaces.srv import StartUDPServer

class AiServiceNode(Node):
    def __init__(self):
        super().__init__('ai_service_node')

        self.create_service(StartUDPServer, 'start_udp_srv', self.handle_start_udp)
        self.create_service(ProductDetection, 'product_detect_srv', self.handle_product_detect)

    def product_detection_callback(self, request, response):
        # Implement your product detection logic here
        response.result = "Detected product: " + request.product_name
        return response

def main():
    print('Hi from ai_server_pkg.')


if __name__ == '__main__':
    main()
