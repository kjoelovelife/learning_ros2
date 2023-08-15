#!/usr/bin/env python3
import math
import rclpy 

from geometry_msgs.msg import Twist
from lesson_interfaces.srv import MoveTurtlesim
from rclpy.node import Node 

class TurtlesimPath:
    Line   = "line"
    Square = "square"
    Circle = "circle"

class MoveTurtlesimServer(Node):
    def __init__(self, node_name: str="move_turtlesim_server_node"):
        super().__init__(node_name)

        self._rate  = self.create_rate(1.0)
        self._twist = Twist()

        self._server = self.create_service(
            srv_type=MoveTurtlesim,
            srv_name="move_turtlesim",
            callback=self.callback_server
        )

        self._publisher = self.create_publisher(
            msg_type=Twist,
            topic="/turtle1/cmd_vel",
            qos_profile=1
        )

    def callback_server(
        self, 
        request: MoveTurtlesim.Request, 
        response: MoveTurtlesim.Response
    ) -> MoveTurtlesim.Response:
        
        path = request.path
        
        response.successful = True
        if path == TurtlesimPath.Line:
            self.turtlesim_move_line()
        elif path == TurtlesimPath.Square:
            self.turtlesim_move_square()
        elif path == TurtlesimPath.Circle:
            self.turtlesim_move_circle()
        else:
            self.turtlesim_stop()
            response.successful = False
        return response

    def sleep(self, sec: float=1.0):
        start_time = self.get_clock().now().nanoseconds
        time_interval  = 0.0
        while(time_interval < sec):
            time_interval = self.get_clock().now().nanoseconds - start_time
            time_interval *= math.pow(10, -9)

    def turtlesim_move_line(self):
        self._twist.linear.x  = 0.2
        self._twist.angular.z = 0.0
        for _ in range(5):
            self._publisher.publish(self._twist)
            self.sleep()
        self.turtlesim_stop()
    
    def turtlesim_move_square(self):
        for _ in range(4):
            self._twist.linear.x  = 2.0
            self._twist.angular.z = 0.0
            self._publisher.publish(self._twist)
            self.sleep()
            self._twist.linear.x  = 0.0
            self._twist.angular.z = math.pi / 2
            self._publisher.publish(self._twist)
            self.sleep()
        self.turtlesim_stop()

    def turtlesim_move_circle(self):
        for _ in range(5):
            self._twist.linear.x  = 1.0
            self._twist.angular.z = math.pi / 2
            self._publisher.publish(self._twist)
            self.sleep()
        self.turtlesim_stop()

    def turtlesim_move_specific_path(self):
        pass

    def turtlesim_stop(self):
        self._twist.linear.x  = 0.0
        self._twist.angular.z = 0.0
        self._publisher.publish(self._twist)

def main(args=None):
    rclpy.init(args=args)
    node = MoveTurtlesimServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__": 
    main()