#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class EvenNumberPublisher(Node):

    def __init__(self):
        super().__init__('even_number_publisher')
        
        # Создаём два издателя с разными топиками
        self.pub_even = self.create_publisher(Int32, 'even_numbers', 10)
        self.pub_overflow = self.create_publisher(Int32, 'overflow', 10)
        
        # Частота 10 Гц = период 0.1 секунды
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.current_number = 0
        self.get_logger().info("Узел запущен! Публикация чётных чисел началась.")

    def timer_callback(self):
        msg = Int32()
        msg.data = self.current_number
        
        # Публикуем число в основной топик
        self.pub_even.publish(msg)
        self.get_logger().info(f'Публикую число: {self.current_number}')
        
        # Проверка на переполнение
        if self.current_number >= 100:
            # Публикуем значение переполнения в отдельный топик
            overflow_msg = Int32()
            overflow_msg.data = self.current_number  # или 100
            self.pub_overflow.publish(overflow_msg)
            
            self.get_logger().warn(f'!!! ПЕРЕПОЛНЕНИЕ !!! Значение: {self.current_number}')
            
            # Сбрасываем счётчик
            self.current_number = 0
        else:
            # Увеличиваем на 2 (чётные числа)
            self.current_number += 2

def main():
    rclpy.init()
    node = EvenNumberPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()