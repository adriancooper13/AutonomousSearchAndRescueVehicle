#!/usr/bin/env python3
import rclpy
import serial

from geometry_msgs.msg import Twist
from math import pi as PI
from rclpy.node import Node

ABS_MAX_PWM = 255
MAX_PWM = 153
MAX_MSEC = 0.9144

class ArduinoController(Node):
    
    def __init__(self):
        super().__init__('arduino_controller')
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.ser.flush()

        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            'cmd_vel',
            self.send_velocity,
            10
        )
        
        self.get_logger().info(f'{self.get_name()} node has started')

    def send_velocity(self, msg: Twist):
        linear = msg.linear.x
        angular = msg.angular.z
        
        left, right = self.math(msg.linear.x, msg.angular.z)

        string = f'{left} {right}' + '\n'
        sent = self.ser.write(string.encode('utf-8'))
        
        line = self.ser.readline().decode('utf-8').rstrip('\n')
        self.get_logger().info(f'{line}')
        
    def math(self, linear, angular):
        total_pwm = int(linear * MAX_PWM / MAX_MSEC)
        x = int(angular * total_pwm / PI)
        
        left_pwm = total_pwm - x
        right_pwm = total_pwm + x
        
        if right_pwm > ABS_MAX_PWM:
            diff = right_pwm - ABS_MAX_PWM
            left_pwm -= diff
            right_pwm = abs_max_diff
            
        if left_pwm > ABS_MAX_PWM:
            diff = left_pwm - ABS_MAX_PWM
            right_pwm -= diff
            left_pwm = ABS_MAX_PWM
            
        return int(left_pwm), int(right_pwm)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ArduinoController())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
