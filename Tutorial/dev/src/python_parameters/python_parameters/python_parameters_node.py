import rclpy
import rclpy.node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType
from rclpy.timer import Timer

class MinimalParam(rclpy.node.Node):

    timer: Timer

    def __init__(self):
        super().__init__('minimal_param_node')
        timer_period = 2 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.declare_parameter('my_parameter', 'world')

    def timer_callback(self):
        my_param = self.get_parameter('my_parameter').get_parameter_value().string_value
        self.get_logger().info(f'Hello {my_param}!')

        my_new_param = rclpy.parameter.Parameter(
            'my_parameter',
            rclpy.Parameter.Type.STRING,
            'world'
        )

        all_new_parameters = [my_new_param]
        self.set_parameters(all_new_parameters)


def main():
    rclpy.init()
    rclpy.spin(MinimalParam())


if __name__ == '__main__':
    main()