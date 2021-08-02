import sys
from typing import Any

import rclpy
from rclpy.client import Client
from rclpy.node import Node
from tutorial_interfaces.srv import AddThreeInts


class MinimalClientAsync(Node):

    cli: Client
    req: Any
    future: Any

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddThreeInts, 'add_three_ints')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.req = AddThreeInts.Request()

    def send_request(self):
        self.req.a = int(sys.argv[1])
        self.req.b = int(sys.argv[2])
        self.req.c = int(sys.argv[3])
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init()

    minimal_client = MinimalClientAsync()
    minimal_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(f'Service call failed {(e,)}')
            else:
                minimal_client.get_logger().info(
                    f'Result of add_three_ints: for {minimal_client.req.a} + {minimal_client.req.b} + {minimal_client.req.c} = {response.sum}'
                )
            
            break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()