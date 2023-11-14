#!/usr/bin/env python3
import math
import rclpy 
import sys
from asyncio import Future
from rclpy.node import Client
from rclpy.node import Node
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import GetParameters
from typing import Any

class TurtlesimBackgroundName:
    BackgroundB = "background_b"
    BackgroundG = "background_g"
    BackgroundR = "background_r"

class GetTurtlesimBackground(Node):
    def __init__(self, node_name: str="get_background_parameter_node"):
        super().__init__(node_name)

        self.__request_parameters_dict = {
            TurtlesimBackgroundName.BackgroundB: 0,
            TurtlesimBackgroundName.BackgroundG: 1,
            TurtlesimBackgroundName.BackgroundR: 2,
        } 
        self.__get_background_parameter_client = self.customic_creat_clinet(
            service_type=GetParameters,
            service_name="get_parameters"
        )

        self.__timer = self.create_timer(
            timer_period_sec=1.0,
            callback=self.callback_timer
        )

    def customic_creat_clinet(self, service_type: Any, service_name: str) -> Client:
        client = self.create_client(
            srv_type=service_type,
            srv_name=service_name
        )
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Wait for service {service_name}...")
        self.get_logger().info(f"Find service {service_name}!")
        return client

    def callback_timer(self):
        request = GetParameters.Request()
        request.names = list(self.__request_parameters_dict.keys())
        future  = self.__get_background_parameter_client.call_async(request)
        future.add_done_callback(self.callback_get_background_parameter_client)

    def callback_get_background_parameter_client(self, future: Future):
        try:
            response: GetParameters.Response = future.result()
        except Exception as error:
            self.get_logger().error(f"Can't retrieve the parameters. {error}")
            sys.exit(1)
        for key, index in self.__request_parameters_dict.items():
            parameter: ParameterValue = response.values[index]
            self.get_logger().info(f"{key}: {parameter.integer_value}")

def main(args=None):
    rclpy.init(args=args)
    get_turtlesim_background_instance = GetTurtlesimBackground()
    
    try:
        rclpy.spin(get_turtlesim_background_instance)
    except KeyboardInterrupt:
        pass
    finally:
        get_turtlesim_background_instance.destroy_node()
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()