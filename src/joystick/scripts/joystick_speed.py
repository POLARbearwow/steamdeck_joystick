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

            # 获取摇杆轴值
            axes = []
            for i in range(joystick.get_numaxes()):
                axes.append(joystick.get_axis(i))

            # 获取按钮值
            buttons = []
            for i in range(joystick.get_numbuttons()):
                buttons.append(joystick.get_button(i))

            # 设置线速度和角速度
            twist = Twist()
            
            # x方向的速度，用第二个摇杆的第二个值控制
            twist.linear.x = axes[1]*-2  # 第二个值控制x轴速度

            # 绕z轴的角速度，用第二个摇杆的第五个值控制
            twist.angular.z = axes[4]*-2  # 第五个值控制绕z轴的角速度

            # 发布速度命令
            self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)

    joystick_node = JoystickNode()
    rclpy.spin(joystick_node)

    joystick_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
