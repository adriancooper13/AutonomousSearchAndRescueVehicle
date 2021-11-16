#!/usr/bin/env python3
import rclpy
import serial

from geometry_msgs.msg import Twist
from math import pi as PI
from rclpy.node import Node
from time import sleep

ABS_MAX_PWM = 255
MAX_PWM = 153
MAX_MSEC = 0.9144

class ArduinoController(Node):
    
    def __init__(self):
        super().__init__('arduino_controller')
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        sleep(5)
        self.ser.flush()

        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            'cmd_vel',
            self.send_velocity,
            10
        )
        
        self.get_logger().info(f'{self.get_name()} node has started')

    def send_velocity(self, msg: Twist):        
        left, right = self.pwm(msg.linear.x, msg.angular.z)

        string = f'{left} {right}' + '\n'
        sent = self.ser.write(string.encode('utf-8'))
        
        line = self.ser.readline().decode('utf-8').rstrip()
        
        self.get_logger().info(f'Bytes Sent: {sent}. Received: {line}')
        
    def pwm(self, linear: float, angular: float):
        if linear != 0.0:
            total_pwm = linear * MAX_PWM / MAX_MSEC
            x = angular * total_pwm / PI
        else:
            total_pwm = 0
            x = MAX_PWM * angular / PI
        
        left_pwm = round(total_pwm - x)
        right_pwm = round(total_pwm + x)
        
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
