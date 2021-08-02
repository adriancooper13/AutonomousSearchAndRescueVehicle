from tutorial_interfaces.srv import AddThreeInts

import rclpy
from rclpy.node import Node
from rclpy.service import Service

class MinimalService(Node):

    srv: Service

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(
            AddThreeInts,
            'add_three_ints',
            self.add_three_ints_callback
        )

    def add_three_ints_callback(self, request, response):
        response.sum = request.a + request.b + request.c
        self.get_logger().info(f'Incoming request [a: {request.a}, b: {request.b}, c: {request.c}]')
        self.get_logger().info(f'Sending response: [{response.sum}]')
        return response


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MinimalService())
    rclpy.shutdown()


if __name__ == '__main__':
    main()