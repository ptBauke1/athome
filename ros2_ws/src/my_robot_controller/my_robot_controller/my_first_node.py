#!/usr/bin/env python3
import rclpy

from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('first_node')
        self.create_timer(1.0, self.timer_callback)
    
    def timer_callback(self):
        self.get_logger().info('Hello ROS 2')
    

def main(args=None):
    # iniciar comunicacao com o ROS 2
    rclpy.init(args=args)
    node = MyNode()
    # ira rodar indefinidamente ate ser interrompido
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
