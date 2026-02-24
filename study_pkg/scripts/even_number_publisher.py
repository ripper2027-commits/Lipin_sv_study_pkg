
#!/usr/bin/env python3
import rclpy                        # Главная библиотека ROS 2 для Python
from rclpy.node import Node         # Базовый класс для узла
from std_msgs.msg import Int8       # Тип сообщения (число)

class Counter(Node):

    def __init__(self):
        super().__init__('counter')
        self.publisher = self.create_publisher(Int8, 'numbers', 10)
        self.publisher = self.create_publisher(Int8, 'overflow', 10)
        timer_period = 1.0        
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.current_number = 0
        self.get_logger().info("Узел counter запущен! Счёт начинается.")

    def timer_callback(self):
        msg = Int8()
        msg.data = self.current_number
        self.publisher.publish(msg)
        self.get_logger().info(f'Число: {self.current_number}')
        self.current_number += 2
        if self.current_number >= 100:
            self.current_number = 0

def main():
    rclpy.init()                    # Инициализация ROS 2
    node = Counter()                # Создание узла

    try:
        rclpy.spin(node)            # Запуск цикла обработки событий
    except KeyboardInterrupt:
        pass                        # Выход по Ctrl+C
    finally:
        node.destroy_node()         # Очистка ресурсов
        rclpy.shutdown()            # Завершение ROS 2

if __name__ == '__main__':
    main()