#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TurtleDrawCircle(Node):
    def __init__(self, node_name: str="turtle_draw_circle"):
        super().__init__(node_name)
        
        self._twist = self.set_speed()

        # === Please modify code below ===

        self._publisher = self.create_publisher(

        )

        self._timer = self.create_timer(

        )

    def set_speed(self) -> Twist:
        twist = Twist()

        return twist

        # === end ===

    def callback_timer(self):
        self._publisher.publish(self._twist)


def main(args=None):
    rclpy.init(args=args)
    turtle_draw_circle_node = TurtleDrawCircle()
    
    try:
        rclpy.spin(turtle_draw_circle_node)
    
    except KeyboardInterrupt:
        pass

    finally:
        turtle_draw_circle_node.destroy_node()
    
    rclpy.shutdown()


if __name__ == "__main__":
    main()

