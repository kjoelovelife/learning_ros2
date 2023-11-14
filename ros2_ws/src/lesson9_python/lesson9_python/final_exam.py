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
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import DescribeParameters
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.srv import SetParametersAtomically
from typing import Any, List

class TurtlesimBackgroundName:
    BackgroundB = "background_b"
    BackgroundG = "background_g"
    BackgroundR = "background_r"

class FinalExam(Node):
    def __init__(self, node_name: str="set_turtelsim_background_node"):
        super().__init__(node_name)

        self.__rgb_upper_bound = 255
        self.__rgb_lower_bound = 0
        self.__velocity_upper_bound = 1.0
        self.__velocity_lower_bound = 0.0
        self.__velocity = 0.0
        self.__has_to_change_color = True

        self.__get_describe_parameters_client = self.customic_creat_clinet(
            service_type=DescribeParameters,
            service_name="describe_parameters"
        )
        self.get_describe_parameters(self.__get_describe_parameters_client, ["velocity"])

        self.__get_velocity_parameter_client = self.customic_creat_clinet(
            service_type=GetParameters,
            service_name="get_parameters"
        )

        self.__set_background_parameters_client = self.customic_creat_clinet(
            service_type=SetParametersAtomically,
            service_name="set_parameters_atomically"
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

    def get_describe_parameters(self, client: Client, names: List[str]):
        request = DescribeParameters.Request()
        request.names = names
        future = client.call_async(request)
        future.add_done_callback(self.callback_get_describe_parameters)

    def callback_get_describe_parameters(self, future: Future):
        try:
            response: DescribeParameters.Response = future.result()
        except Exception as error:
            self.get_logger().error(f"Call service failed. {error}")
            sys.exit(1)
        velocity_parameter_descriptor: ParameterDescriptor = response.descriptors[0]
        velocity_range: FloatingPointRange = velocity_parameter_descriptor.floating_point_range[0]
        self.__velocity_upper_bound = velocity_range.to_value
        self.__velocity_lower_bound = velocity_range.from_value

    def callback_timer(self):
        request = GetParameters.Request()
        request.names = ["velocity"]
        future  = self.__get_velocity_parameter_client.call_async(request)
        future.add_done_callback(self.callback_get_velocity_parameter_client)

        if self.__has_to_change_color:
            velocity_ratio = (self.__velocity + math.fabs(self.__velocity_lower_bound)) \
                / (self.__velocity_upper_bound + math.fabs(self.__velocity_lower_bound))
            rgb_value = (self.__rgb_upper_bound - self.__rgb_lower_bound) * velocity_ratio
            request = SetParametersAtomically.Request()
            request.parameters = [
                Parameter(
                    name=TurtlesimBackgroundName.BackgroundR,
                    value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=round(rgb_value))
                )
            ]
            future: Future = self.__set_background_parameters_client.call_async(request)
            future.add_done_callback(self.callback_set_background_parameters_client)

    def callback_get_velocity_parameter_client(self, future: Future):
        try:
            response: GetParameters.Response = future.result()
        except Exception as error:
            self.get_logger().error(f"Can't retrieve the parameters. {error}")
            sys.exit(1)
        velocity_from_parameter_server: ParameterValue = response.values[0]
        if self.__velocity == velocity_from_parameter_server.double_value:
            self.__has_to_change_color = False
        else:
            self.__has_to_change_color = True
            self.__velocity = velocity_from_parameter_server.double_value

    def callback_set_background_parameters_client(self, future: Future):
        try:
            response: SetParametersAtomically.Response = future.result()
            if response.result.successful:
                self.get_logger().info(f"Change background!")
        except Exception as error:
            self.get_logger().error(f"Could not set the background! {error}")
            sys.exit(1)

def main(args=None):
    rclpy.init(args=args)
    final_exam_instance = FinalExam()
    
    try:
        rclpy.spin(final_exam_instance)
    except KeyboardInterrupt:
        pass
    finally:
        final_exam_instance.destroy_node()
    
    rclpy.shutdown()


if __name__ == "__main__":
    main()