import random
import rclpy

from custom_interfaces.srv import TransferGolfballLocations
from gazebo_msgs.srv import SpawnEntity
from .filepaths import model_paths
from .functions import euler_to_quaternion, feet_to_meters
from math import pi as PI
from rclpy.client import Client
from rclpy.qos import HistoryPolicy, QoSProfile
from rclpy.node import Node
from rclpy.task import Future

class CreateWorld(Node):

    client: Client
    golfballs: list

    def __init__(self):
        super().__init__('world_creator')
        self.client = self.create_client(
            srv_type=SpawnEntity,
            srv_name='/spawn_entity',
            qos_profile=QoSProfile(history=HistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_ALL)
        )
        while not self.client.wait_for_service(1.0):
            self.get_logger().warn('Service not available. Waiting...')

        self.get_logger().info(f'{self.get_name()} node has started')
        self.golfballs = []
        self.create_world()
        self.publish_golfballs()

    def publish_golfballs(self):
        def publish_golfballs_results(future: Future):
            try:
                response = future.result()
                if not response.success:
                    raise Exception()
            except Exception as e:
                self.get_logger().fatal(f'Could not send golf ball poses. Exception: {e}')

        client = self.create_client(TransferGolfballLocations, 'golfball_locations')
        while not client.wait_for_service(1.0):
            self.get_logger().info(f'Waiting for {client.srv_name} service...')

        request = TransferGolfballLocations.Request()
        request.xs, request.ys, request.names = zip(*self.golfballs)

        future = client.call_async(request)
        future.add_done_callback(publish_golfballs_results)

    def create_world(self):
        self.spawn_robot()
        self.spawn_golfballs()

    def spawn(self, name: str, xml: str, pose: dict):
        def spawn_result(future: Future):
            try:
                result = future.result()
                self.get_logger().info(f'Result: {result}')
            except Exception as e:
                self.get_logger().warn(f'Could not spawn {name}. Exception: {e}')
        
        request = SpawnEntity.Request()
        request.name = name
        request.xml = xml
        request.initial_pose.position.x = float(pose['x'])
        request.initial_pose.position.y = float(pose['y'])
        request.initial_pose.position.z = float(pose['z'])
        try:
            orientation = pose['orientation']
            request.initial_pose.orientation.x = float(orientation['x'])
            request.initial_pose.orientation.y = float(orientation['y'])
            request.initial_pose.orientation.z = float(orientation['z'])
            request.initial_pose.orientation.w = float(orientation['w'])
        except KeyError:
            pass

        future = self.client.call_async(request)
        future.add_done_callback(spawn_result)

    def spawn_golfballs(self):
        num_golfballs = random.randint(30, 60)
        pose = {'z': 0.0}
        dist = feet_to_meters(75)
        for i in range(num_golfballs):
            while True:
                pose['x'] = (random.random() * dist) - (dist / 2)
                pose['y'] = (random.random() * dist) - (dist / 2)
                # Make sure golf balls do not spawn in the dropoff area.
                if pose['y'] > -8.430388 or pose['x'] < 8.421658:
                    break

            name = f'ball{i}'

            golfball_info = (pose['x'], pose['y'], name)
            self.golfballs.append(golfball_info)

            self.spawn(
                name=name,
                xml=open(model_paths['golfball']).read(),
                pose=pose
            )

    def spawn_robot(self):
        self.spawn(
            name='turtlebot',
            xml=open(model_paths['robot']).read(),
            pose={'x': 11, 'y': -11, 'z': 0, 'orientation': euler_to_quaternion(0, 0, PI)}
        )

def main():
    rclpy.init()
    rclpy.spin_once(CreateWorld())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
