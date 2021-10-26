import rclpy

from custom_interfaces.srv import TransferGolfballLocations as GolfballLocations
from gazebo_msgs.srv import DeleteEntity
from geometry_msgs.msg import Point
from math import sqrt
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.service import Service
from rclpy.subscription import Subscription
from rclpy.task import Future
from time import sleep
from typing import List

class Golfball:
    x: float
    y: float
    name: str

    def __init__(self, x: float, y: float, name: str):
        self.x = x
        self.y = y
        self.name = name

    def distance(self, x: float, y: float):
        return sqrt((self.x - x) ** 2 + (self.y - y) ** 2)

class RemoveGolfballs(Node):
    
    golfball_locations: Service
    odom_subscription: Subscription
    golfballs: List[Golfball]

    def __init__(self):
        super().__init__('remove_golfballs')

        sleep(20)
        self.golfballs = None
        self.golfball_locations = self.create_service(
            GolfballLocations,
            'golfball_locations',
            self.get_golfball_locations
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.check_position,
            10
        )

        self.get_logger().info(f'{self.get_name()} node has started')

    def check_position(self, msg: Odometry):
        if self.golfballs is None:
            return

        position = msg.pose.pose.position
        for i, gb in enumerate(self.golfballs):
            if gb.distance(position.x, position.y) < 0.2:
                self.remove_golfball(gb, i)
                break

    def remove_golfball(self, gb: Golfball, index: int):
        def remove_golfball_result(future: Future):
            try:
                response = future.result()
                if response.success:
                    self.golfballs.pop(index)
                self.get_logger().info(f'Deleted golfball: {response.success}')
            except Exception as e:
                self.get_logger().warn(f'Service call failed. Exception: {e}')

        request = DeleteEntity.Request()
        request.name = gb.name

        client = self.create_client(DeleteEntity, 'delete_entity')
        while not client.wait_for_service(1.0):
            self.get_logger().warn(f'{client.srv_name} service is not available...')

        future = client.call_async(request)
        future.add_done_callback(remove_golfball_result)

    def get_golfball_locations(self, request, response):
        if len(request.xs) != len(request.ys) or len(request.xs) != len(request.names):
            response.success = False
            return response
        
        length = len(request.xs)
        self.golfballs = [
            Golfball(request.xs[i], request.ys[i], request.names[i]) for i in range(length)
        ]

        response.success = True
        self.get_logger().info('Received golfball locations')
        return response


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(RemoveGolfballs())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
