#!/usr/bin/env pytohn3
import rclpy
from rclpy.node import Node
from lesson_interfaces.msg import Lunch
class LunchInfoSubscriber(Node):
    def __init__(self, node_name: str="lunch_info_subscriber"):
        super().__init__(node_name)

        self._subscriber = self.create_subscription(
            msg_type=Lunch,
            topic="lunch_info",
            callback=self.callback_subscriber,
            qos_profile=1
        )

    def callback_subscriber(self, msg: Lunch):
        content = f"""
        How many bowls of rice do you use? {msg.bowls_of_rice}
        What's the color of bowls? {msg.color_of_bowls}
        What types of the meats do you eat? {msg.meats}
        What types of the vegetables do you eat? {msg.vegetables}
        """
        self.get_logger().info(content)

def main(args=None):
    rclpy.init(args=args)
    subscriber = LunchInfoSubscriber()
    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()