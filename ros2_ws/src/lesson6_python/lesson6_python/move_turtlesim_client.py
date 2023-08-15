#!/usr/bin/env python3
import rclpy 

from asyncio import Future
from lesson_interfaces.srv import MoveTurtlesim
from rclpy.node import Node 

class TurtlesimPath:
    Line   = "line"
    Square = "square"
    Circle = "circle"

class MoveTurtlesomClient(Node):
    def __init__(self, node_name: str="move_turtlesim_client_node"):
        super().__init__(node_name)

        self._client = self.create_client(
            srv_type=MoveTurtlesim,
            srv_name="move_turtlesim",
        )
        while not self._client.wait_for_service(1.0):
            self.get_logger().info(f"Wait for the \"{self._client.srv_name}\" service...")


        self.call_service_async(path=TurtlesimPath.Line)
        self.call_service_async(path=TurtlesimPath.Square)
        self.call_service_async(path=TurtlesimPath.Circle)

    def call_service_async(self, path: str):
        request = MoveTurtlesim.Request()
        request.path = path
        self.get_logger().info(f"Call Turtlesim to move {path}")
        future: Future = self._client.call_async(request)
        future.add_done_callback(self.callback_service)

    def callback_service(self, future: Future):
        response: MoveTurtlesim.Response = future.result()
        if response.successful:
            self.get_logger().info(f"Turtlesim Move done!")
        else:
            self.get_logger().error(f"Something error! Turtlesim stop!")


def main(args=None):
    rclpy.init(args=args)
    node = MoveTurtlesomClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__": 
    main()