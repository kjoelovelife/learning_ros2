#!/usr/bin/env python3
import math
import rclpy 
import sys
from asyncio import Future
from rclpy.node import Client
from rclpy.node import Node
from rcl_interfaces.msg import FloatingPointRange
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import DescribeParameters
from rcl_interfaces.srv import SetParametersAtomically
from typing import Any, Dict

class SetTurtleVelocity(Node):
    def __init__(self, node_name: str="set_turtelsim_background_node"):
        super().__init__(node_name)

        self.__velocity_upper_bound = 1.0
        self.__velocity_lower_bound = 0.0
        self.__velocity = 0.0
 
        self.__get_describe_parameters_client = self.customic_create_clinet(
            service_type=DescribeParameters,
            service_name="describe_parameters"
        )
        self.__request_parameter_dict = {"velocity": 0}
        self.set_velocity_bound(self.__get_describe_parameters_client, self.__request_parameter_dict)
        
        self.__set_turtle_velocty_client = self.customic_create_clinet(
            service_type=SetParametersAtomically,
            service_name="set_parameters_atomically"
        )

        self.__control_frequency = 20 # Hz
        self.__acceleration = 0.001 # m/s^2
        self.__timer = self.create_timer(
            timer_period_sec=(1.0/self.__control_frequency),
            callback=self.callback_timer
        )

    def customic_create_clinet(self, service_type: Any, service_name: str) -> Client:
        client = self.create_client(
            srv_type=service_type,
            srv_name=service_name
        )
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Wait for service {service_name}...")
        self.get_logger().info(f"Find service {service_name}!")
        return client

    def set_velocity_bound(self, client: Client, names_dcit: Dict[str, int]):
        request = DescribeParameters.Request()
        request.names = list(names_dcit.keys())
        future = client.call_async(request)
        future.add_done_callback(self.callback_get_describe_parameters)

    def callback_get_describe_parameters(self, future: Future):
        try:
            response: DescribeParameters.Response = future.result()
        except Exception as error:
            self.get_logger().error(f"Call service failed. {error}")
            sys.exit(1)
        velocity_parameter_descriptor: ParameterDescriptor = \
            response.descriptors[self.__request_parameter_dict["velocity"]]
        velocity_range: FloatingPointRange = velocity_parameter_descriptor.floating_point_range[0]
        self.__velocity_upper_bound = velocity_range.to_value
        self.__velocity_lower_bound = velocity_range.from_value

    def callback_timer(self):
        if (self.__velocity > self.__velocity_upper_bound) or (self.__velocity < self.__velocity_lower_bound):
            self.__acceleration = -self.__acceleration

        velocity_parameter = Parameter()
        velocity_parameter.name = "velocity"
        velocity_parameter.value.type = ParameterType.PARAMETER_DOUBLE
        self.__velocity = self.__velocity + self.__acceleration*self.__control_frequency
        velocity_parameter.value.double_value = self.__velocity

        request = SetParametersAtomically.Request()
        request.parameters = [velocity_parameter]
        future  = self.__set_turtle_velocty_client.call_async(request)
        future.add_done_callback(self.callback_set_turtle_velocty_client)

    def callback_set_turtle_velocty_client(self, future: Future):
        try:
            response: SetParametersAtomically.Response = future.result()
            if response.result.successful:
                self.get_logger().info(f"Now the turtle's speed is {self.__velocity:.3f} m/s")
        except Exception as error:
            self.get_logger().error(f"Could not set the turtle's speed! {error}")
            sys.exit(1)

def main(args=None):
    rclpy.init(args=args)
    set_turtle_velocity_instance = SetTurtleVelocity()
    
    try:
        rclpy.spin(set_turtle_velocity_instance)
    except KeyboardInterrupt:
        pass
    finally:
        set_turtle_velocity_instance.destroy_node()
    
    rclpy.shutdown()


if __name__ == "__main__":
    main()