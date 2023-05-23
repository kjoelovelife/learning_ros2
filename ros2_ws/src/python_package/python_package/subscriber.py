#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Subscriber(Node):
    def __init__(self, node_name: str="subscriber_node"):
        super().__init__(node_name)
        self._subscriber = self.create_subscription(
            msg_type=String,
            topic="sentence",
            callback=self.callback_subscriber,
            qos_profile=10
        )


    def callback_subscriber(self, msg: String):
        words = msg.data
        self.get_logger().info(f"I heard {words}")


def main(args=None):
    rclpy.init(args=args)
    subscriber = Subscriber()
    rclpy.spin(subscriber)
    rclpy.shutdown()


if __name__ == "__main__":
    main()