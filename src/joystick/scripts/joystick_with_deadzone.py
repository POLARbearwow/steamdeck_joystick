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

        dead_zone_threshold = 2000  # Set the dead zone threshold (absolute value)

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

                # Apply dead zone to axes 1, 2, 3, and 4 (index 0, 1, 2, and 3)
                if i in [0, 1, 3, 4]:  # These correspond to axes 1, 2, 3, and 4
                    if abs(axis_value) < dead_zone_threshold / 32767.0:  # Normalize the threshold
                        axis_value = 0.0  # Set to zero if within the dead zone

                axes.append(axis_value)

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
