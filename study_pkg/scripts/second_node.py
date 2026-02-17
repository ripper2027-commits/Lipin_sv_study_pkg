#!/usr/bin/env python3
"""Первый узел ROS 2 — Hello World"""

import rclpy
import time
from datetime import datetime
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args)                   # инициализация ROS 2
    node = Node('node1')
    while True:
        now = datetime.now()
        current_time = now.strftime("%H:%M:%S")                          # создаём узел с именем hello_node
        node.get_logger().info(current_time)
        time.sleep(5) 
    rclpy.spin(node)                        # запускаем цикл обработки
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()