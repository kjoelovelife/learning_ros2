#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TurtleDrawCircle(Node):
    def __init__(self, node_name: str="turtle_draw_circle"):
        super().__init__(node_name)
        
        self._twist = self.set_speed()

        self._publisher = self.create_publisher(
            msg_type=Twist,
            topic="/turtle1/cmd_vel",
            qos_profile=10
        )

        self._timer = self.create_timer(
            timer_period_sec=0.5,
            callback=self.callback_timer
        )


    def set_speed(self) -> Twist:
        twist = Twist()
        twist.linear.x = 1.0
        twist.angular.z = 1.0
        return twist

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

