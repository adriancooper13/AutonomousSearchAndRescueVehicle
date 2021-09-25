import random
import rclpy

from gazebo_msgs.srv import SpawnEntity
from helpers.filepaths import model_paths
from helpers.functions import euler_to_quaternion, feet_to_meters
from math import radians
from rclpy.qos import HistoryPolicy, QoSProfile
from rclpy.node import Node
from rclpy.task import Future

class CreateWorld(Node):

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
        self.create_world()

    def create_world(self):
        self.spawn_grass()
        self.spawn_tape()
        # self.spawn_border()
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

    def spawn_grass(self):
        self.spawn(
            name='grass',
            xml=open(model_paths['grass']).read(),
            pose={'x': 0, 'y': 0, 'z': 0}
        )
    
    def spawn_tape(self):
        coordinates = [
            {'x': 9.898041, 'y': -8.527939, 'z': 0.0},
            {'x': 8.521017, 'y': -9.901380, 'z': 0.0, 'orientation': euler_to_quaternion(0, 0, radians(90))}
        ]
        for i in range(2):
            self.spawn(
                name=f'tape{i}',
                xml=open(model_paths['tape']).read(),
                pose=coordinates[i]
            )

    def spawn_golfballs(self):
        num_golfballs = random.randint(30, 60)
        pose = {'z': 0.0}
        dist = feet_to_meters(75)
        for i in range(num_golfballs):
            while True:
                pose['x'] = (random.random() * dist) - (dist / 2)
                pose['y'] = (random.random() * dist) - (dist / 2)
                # TODO: make sure golf balls do not spawn in the dropoff area
                # if pose['x'] < 8.521017 and pose['y'] < -9.901380:
                #     break
                break

            self.spawn(
                name=f'ball{i}',
                xml=open(model_paths['golfball']).read(),
                pose=pose
            )

    def spawn_robot(self):
        self.spawn(
            name='turtlebot',
            xml=open(model_paths['robot']).read(),
            pose={'x': 11, 'y': -11, 'z': 0, 'orientation': euler_to_quaternion(0, 0, radians(180))}
        )

    def spawn_border(self):
        DISTANCE_TO_EDGE = round(feet_to_meters(75), 2)
        BORDER_WIDTH = 0.3
        PLACEMENT = (DISTANCE_TO_EDGE + BORDER_WIDTH) / 2

        poses = [
            {'x': 0, 'y': PLACEMENT, 'z': 0},
            {'x': PLACEMENT, 'y': 0, 'z': 0, 'orientation': euler_to_quaternion(0, 0, radians(90))},
            {'x': 0, 'y': -PLACEMENT, 'z': 0},
            {'x': -PLACEMENT, 'y': 0, 'z': 0, 'orientation': euler_to_quaternion(0, 0, radians(90))}
        ]
        
        for i, pose in enumerate(poses):
            self.spawn(
                name=f'border{i}',
                xml=open(model_paths['border']).read(),
                pose=pose
            )
        


def main():
    rclpy.init()
    rclpy.spin_once(CreateWorld())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
