#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from lesson_interfaces.msg import Lunch

class Purple:
    R = 106.0
    G = 90.0
    B = 205.0
    A = 1.0

class LunchInfoPublisher(Node):
    def __init__(self, node_name: str="lunch_info_publisher"):
        super().__init__(node_name)
        self._publisher = self.create_publisher(
            msg_type=Lunch,
            topic="lunch_info",
            qos_profile=1
        )

        self.lunch = Lunch()
        self.lunch.bowls_of_rice = 2
        color = Purple()
        self.lunch.color_of_bowls.r = color.R
        self.lunch.color_of_bowls.g = color.G
        self.lunch.color_of_bowls.b = color.B
        self.lunch.color_of_bowls.a = color.A
        self.lunch.meats = ["fish", "pork"]
        self.lunch.vegetables = ["spinach", "tomato"]
        self._timer = self.create_timer(
            timer_period_sec=0.5,
            callback=self.callback_timer
        )

    def callback_timer(self):
        self._publisher.publish(self.lunch)

def main(args=None):
    rclpy.init(args=args)
    publihser= LunchInfoPublisher()

    try:
        rclpy.spin(publihser)
    except KeyboardInterrupt:
        pass
    finally:
        publihser.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()

