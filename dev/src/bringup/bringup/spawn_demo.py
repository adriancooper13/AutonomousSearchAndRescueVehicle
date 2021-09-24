import os
import random
import rclpy

from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
from math import radians, sin, cos
from rclpy.qos import HistoryPolicy, QoSProfile
from rclpy.node import Node
from rclpy.task import Future


feet_to_meters = lambda feet: feet * 0.3048

# roll (X), pitch (Y), yaw (Z)
# All arguemnts are assumed to be in radians
def euler_to_quaternion(roll: float, pitch: float, yaw: float):
    # Abbreviations for the various angular functions
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)

    return {
        'w': cr * cp * cy + sr * sp * sy,
        'x': sr * cp * cy - cr * sp * sy,
        'y': cr * sp * cy + sr * cp * sy,
        'z': cr * cp * sy - sr * sp * cy
    }

directories = {
    'package': 'bringup',
    'models': 'gazebo/models'
}
models = {
    'grass': 'grass_plane',
    'golfball': 'GolfBall',
    'tape': 'DropOffTape'
}
filenames = {
    'model': 'model.sdf'
}
model_paths = {
    'grass': os.path.join(
        get_package_share_directory(directories['package']),
        directories['models'],
        models['grass'],
        filenames['model']
    ),
    'golfball': os.path.join(
        get_package_share_directory(directories['package']),
        directories['models'],
        models['golfball'],
        filenames['model']
    ),
    'tape': os.path.join(
        get_package_share_directory(directories['package']),
        directories['models'],
        models['tape'],
        filenames['model']
    )
}

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
            request.initial_pose.orientation.x = orientation['x']
            request.initial_pose.orientation.y = orientation['y']
            request.initial_pose.orientation.z = orientation['z']
            request.initial_pose.orientation.w = orientation['w']
        except KeyError:
            pass

        future = self.client.call_async(request)
        future.add_done_callback(spawn_result)

    def spawn_grass(self):
        self.spawn(
            name='grass',
            xml=open(model_paths['grass']).read(),
            pose={'x': 40.0, 'y': 40.0, 'z': 0.0}
        )
    
    def spawn_tape(self):
        coordinates = [
            {'x': 9.898041, 'y': -8.527939, 'z': 0.0},
            # For some reason the yaw is not correct when transferred to gazebo.
            # This really rotates it (about) 90 degrees.
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
                # if pose['x'] < 8.521017 and pose['y'] < -9.901380:
                #     break
                break

            self.spawn(
                name=f'ball{i}',
                xml=open(model_paths['golfball']).read(),
                pose=pose
            )        


def main():
    rclpy.init()
    rclpy.spin(CreateWorld())
    rclpy.shutdown()


if __name__ == '__main__':
    main()