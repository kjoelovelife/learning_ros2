#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Publisher(Node):
    def __init__(self, node_name: str="publisher_node"):
        super().__init__(node_name)
        self._publisher = self.create_publisher(
            msg_type=String,
            topic="sentence",
            qos_profile=10
        )

        self.word = String()
        self.word.data = "Hello, ROS 2! I'll make an interesting!"
        self._timer = self.create_timer(
            timer_period_sec=1.0,
            callback=self.callback_timer
        )

    def callback_timer(self):
        self._publisher.publish(self.word)

def main(args=None):
    rclpy.init(args=args)
    publisher = Publisher()
    
    try:
        rclpy.spin(publisher)
    
    except KeyboardInterrupt:
        pass

    finally:
        publisher.destroy_node()
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()
