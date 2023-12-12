#!/usr/bin/env python3
import math
import rclpy
from copy import deepcopy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import FloatingPointRange
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class RetrieveLaserScan(Node):
    def __init__(self, node_name: str="retrieve_laser_scan_node"):
        super().__init__(node_name)

        self.__laser_scan_subscriber = self.create_subscription(
            msg_type=LaserScan,
            topic="scan",
            qos_profile=10,
            callback=self.__callback_laser_scan_subscriber
        )

        self.__laser_scan_publisher = self.create_publisher(
            msg_type=LaserScan,
            topic="custom_scan",
            qos_profile=10
        )

        self.__scan_angle_range_parameter = "scan_angle_range"
        scan_angle_parameter_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER,
            description=f"Specify the angle range for lidar scanning",
            floating_point_range=[FloatingPointRange(
                from_value=0.0,
                to_value=270.0,
                step=0.001
            )]
        )
        self.declare_parameter(
            name=self.__scan_angle_range_parameter, 
            value=270.0,
            descriptor=scan_angle_parameter_descriptor
        )

    def __callback_laser_scan_subscriber(self, msg: LaserScan):
        angle_range = self.get_parameter(self.__scan_angle_range_parameter).value
        radian_range = len(msg.ranges)*msg.angle_increment if angle_range >= 270.0 \
            else angle_range*math.pi/180.0
        
        max_index = math.floor(radian_range / msg.angle_increment)
        publish_msg = deepcopy(msg)
        publish_msg.ranges = publish_msg.ranges[:max_index]    
        self.__laser_scan_publisher.publish(publish_msg)


def main(args=None):
    rclpy.init(args=args)
    retrieve_laser_scan_instance = RetrieveLaserScan()

    try:
        rclpy.spin(retrieve_laser_scan_instance)
    except KeyboardInterrupt:
        pass
    finally:
        retrieve_laser_scan_instance.destroy_node()
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()