#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import pygame

class JoystickNode(Node):
    def __init__(self):
        super().__init__('joystick_node')
        self.publisher_ = self.create_publisher(Joy, 'joystick', 10)
        self.get_joystick_input()

    def get_joystick_input(self):
        pygame.init()
        pygame.joystick.init()

        # Assuming that you have only one joystick connected
        joystick = pygame.joystick.Joystick(0)
        joystick.init()

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
                axes.append(joystick.get_axis(i))

            buttons = []
            for i in range(joystick.get_numbuttons()):
                buttons.append(joystick.get_button(i))
                
            joy = Joy()
            joy.axes = axes
            joy.buttons = buttons
            self.publisher_.publish(joy)

def main(args=None):
    rclpy.init(args=args)

    joystick_node = JoystickNode()
    rclpy.spin(joystick_node)
    joystick_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
