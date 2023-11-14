#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import FloatingPointRange
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType
from geometry_msgs.msg import Twist

class TurtleDrawCircle(Node):
    def __init__(self, node_name: str="turtle_draw_circle"):
        super().__init__(node_name)
        
        self.__velocity_name = "velocity"
        self.declare_ros2_parameter(self.__velocity_name)
        self.__publisher = self.create_publisher(
            msg_type=Twist,
            topic="cmd_vel",
            qos_profile=10
        )

        self.__timer = self.create_timer(
            timer_period_sec=0.5,
            callback=self.callback_timer
        )

    def declare_ros2_parameter(self, parameter_name: str):
        parameter_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description=f"Set the {parameter_name} of the turtlesim.",
            floating_point_range=[FloatingPointRange(
                from_value=-1.0,
                to_value=1.0,
                step=0.001
            )]
        )
        self.declare_parameter(
            name=parameter_name,
            value=0.0,
            descriptor=parameter_descriptor
        )

    def set_speed(self, parameter_name: str) -> Twist:
        twist = Twist()
        twist.linear.x = self.get_parameter(parameter_name).value
        twist.angular.z = self.get_parameter(parameter_name).value
        return twist

    def callback_timer(self):
        twist = self.set_speed(self.__velocity_name)
        self.__publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    turtle_draw_circle_instance = TurtleDrawCircle()
    
    try:
        rclpy.spin(turtle_draw_circle_instance)
    
    except KeyboardInterrupt:
        pass

    finally:
        turtle_draw_circle_instance.destroy_node()
    
    rclpy.shutdown()


if __name__ == "__main__":
    main()

