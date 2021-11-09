#!/usr/bin/env python3
import serial
import rclpy

from geometry_msgs.msg import Twist
from rclpy.node import Node

class ArduinoController(Node):
    
    def __init__(self):
        super().__init__('arduino_controller')

        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        self.ser.flush()

        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            'cmd_vel',
            self.send_velocity,
            10
        )

    def send_velocity(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        string = f'{linear} {angular}\n'
        self.ser.write(string.encode('utf-8'))

        line = self.ser.readline().decode('utf-8').rstrip()
        self.get_logger().info(f'{line}')


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ArduinoController())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
