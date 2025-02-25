#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import pygame

class JoystickNode(Node):
    def __init__(self):
        super().__init__('joystick_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_joystick_input()

    def get_joystick_input(self):
        pygame.init()
        pygame.joystick.init()

        joystick = pygame.joystick.Joystick(0)
        joystick.init()

        dead_zone_threshold = 0.2  # Set the dead zone threshold (you can adjust this value)

        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    done = True
                
                elif event.type == pygame.JOYBUTTONDOWN:
                    print("Joystick button pressed.")
                elif event.type == pygame.JOYBUTTONUP:
                    print("Joystick button released.")

            axes = []
            for i in range(joystick.get_numaxes()):
                axis_value = joystick.get_axis(i)

                if abs(axis_value) < dead_zone_threshold:
                    axis_value = 0.0

                axes.append(axis_value)

            buttons = []
            for i in range(joystick.get_numbuttons()):
                buttons.append(joystick.get_button(i))

            twist = Twist()

            
            twist.linear.x = axes[1] * -2  # 第二个值控制x轴速度

            twist.angular.z = axes[4] * -2  # 第五个值控制绕z轴的角速度

            self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)

    joystick_node = JoystickNode()
    rclpy.spin(joystick_node)

    joystick_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
