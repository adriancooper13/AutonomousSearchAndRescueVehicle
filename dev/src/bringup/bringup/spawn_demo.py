import os
import random
import rclpy
import rclpy.qos

from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
from rclpy.node import Node
from rclpy.task import Future

feet_to_meters = lambda feet: feet * 0.3048

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
        qos = rclpy.qos.QoSProfile(history = rclpy.qos.HistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_ALL)
        qos.history = rclpy.qos.HistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_ALL
        self.client = self.create_client(SpawnEntity, '/spawn_entity', qos_profile=qos)
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

        if 'tape' in name:
            xml.replace('<allow_auto_disable>1</allow_auto_disable>', '<allow_auto_disable>0</allow_auto_disable>')
            # self.get_logger().info(xml)
        
        request = SpawnEntity.Request()
        request.name = name
        request.xml = xml
        request.initial_pose.position.x = float(pose['x'])
        request.initial_pose.position.y = float(pose['y'])
        request.initial_pose.position.z = float(pose['z'])

        future = self.client.call_async(request)
        future.add_done_callback(spawn_result)

    def spawn_grass(self):
        self.spawn(
            name='grass',
            xml=open(model_paths['grass']).read(),
            pose={'x': 40.0, 'y': 40.0, 'z': 0.0}
        )
    
    def spawn_tape(self):
        self.spawn(
            name='tape1',
            xml=open(model_paths['tape']).read(),
            pose={'x': 9.898041, 'y': -8.527939, 'z': 0}
        )
        self.spawn(
            name='tape2',
            xml=open(model_paths['tape']).read(),
            pose={'x': 8.521017, 'y': -9.901380, 'z': 0}
        )

    def spawn_golfballs(self):
        num_golfballs = random.randint(30, 60)
        pose = {'z': 0}
        dist = round(feet_to_meters(75), 2)
        for i in range(num_golfballs):
            while True:
                pose['x'] = random.random() * dist - (dist / 2)
                pose['y'] = random.random() * dist - (dist / 2)
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