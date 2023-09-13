#!/usr/bin/env python3

import math
import rclpy
import sys
import select
import termios
import tty

from rclpy.node import Node
from geometry_msgs.msg import Twist
from typing import Union, List


class TeleopInTerminal(Node):
    def __init__(self, node_name: str="teleop_in_terminal"):
        super().__init__(node_name)

        self.__twist     = Twist()
        self.__publisher = self.create_publisher(
            msg_type=Twist,
            topic="/turtle1/cmd_vel",
            qos_profile=10
        )

    def get_key(self, settings: Union[List, None]) -> str:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def forward(self):
        self.__twist.linear.x  = 1.0
        self.__twist.angular.z = 0.0
        self.__publisher.publish(self.__twist)

    def backword(self):
        self.__twist.linear.x  = -1.0
        self.__twist.angular.z = 0.0
        self.__publisher.publish(self.__twist)

    def turn_left(self):
        self.__twist.linear.x  = 0.0
        self.__twist.angular.z = math.pi / 2
        self.__publisher.publish(self.__twist)

    def turn_right(self):
        self.__twist.linear.x  = 0.0
        self.__twist.angular.z = -math.pi / 2
        self.__publisher.publish(self.__twist)

    def stop(self):
        self.__twist.linear.x  = 0.0
        self.__twist.angular.z = 0.0
        self.__publisher.publish(self.__twist)

def main(args=None):
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    teleop_in_terminal_node = TeleopInTerminal()
    msg = """
Control the Turtle!
-------------------------
Moving around:
    w
a   s   d
    x
    
w/x: moving the turtle forward/backward
a/d: turning the turtle left/right
s  : force stop the turtle
"""
    try:
        print(msg)
        while True:
            key = teleop_in_terminal_node.get_key(settings)
            if key == "w":
                teleop_in_terminal_node.forward()
            elif key == "x":
                teleop_in_terminal_node.backword()
            elif key == "a":
                teleop_in_terminal_node.turn_left()
            elif key == "d":
                teleop_in_terminal_node.turn_right()
            elif key == "s":
                teleop_in_terminal_node.stop()
            else:
                teleop_in_terminal_node.stop()
                if (key == "\x03"):
                    break
    except Exception:
        print("Communications Failed!")
    
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            
if __name__ == "__main__":
    main()