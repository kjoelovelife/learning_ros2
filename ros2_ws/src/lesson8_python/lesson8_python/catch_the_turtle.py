#!/usr/bin/env python3

import math
import rclpy 
from asyncio import Future
from functools import partial
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from lesson_interfaces.msg import Turtle
from lesson_interfaces.msg import TurtleArray
from lesson_interfaces.srv import CatchTurtle
from rclpy.node import Node


class CatchTheTurtle(Node):
    def __init__(self, node_name: str="catch_the_turtle_node"):
        super().__init__(node_name)

        self.__turtle_to_catch = None
        self.__pose = None
        self.__p_controller_coefficient = 1.0


        # Create Publisher
        self.__cmd_vel_publisher = self.create_publisher(
            msg_type=Twist,
            topic="/turtle1/cmd_vel",
            qos_profile=10
        )

        # Create Subscriber
        self.__pose_subscriber = self.create_subscription(
            msg_type=Pose,
            topic="/turtle1/pose",
            callback=self.__callback_turtle_pose,
            qos_profile=10
        )

        self.__alive_turtles_subscriber = self.create_subscription(
            msg_type=TurtleArray,
            topic="alive_turtles",
            callback=self.__callback_alive_turtles,
            qos_profile=10
        )

        # Create timer
        self.__control_loop_time = self.create_timer(
            timer_period_sec=0.01, 
            callback=self.__controller_loop
        )

    def __callback_turtle_pose(self, msg: Pose):
        self.__pose = msg

    def __callback_alive_turtles(self, msg: TurtleArray):
        if len(msg.turtle_array) > 0:
            self.__turtle_to_catch = msg.turtle_array[0]

    def __controller_loop(self):
        if self.__pose == None or self.__turtle_to_catch == None:
            return
        self.__turtle_to_catch: Turtle
        self.__pose: Pose
        distance_x = self.__turtle_to_catch.pose.x - self.__pose.x
        distance_y = self.__turtle_to_catch.pose.y - self.__pose.y
        distance = math.sqrt(math.pow(distance_x, 2)+math.pow(distance_y, 2))

        twist_msg = Twist()

        # Use P controller
        if distance > 0.5:
            twist_msg.linear.x = self.__p_controller_coefficient * 0.5 * distance
            goal_theta = math.atan2(distance_y, distance_x)
            theta_to_goal = goal_theta - self.__pose.theta
            if theta_to_goal > math.pi:
                theta_to_goal -= (2*math.pi)
            elif theta_to_goal < -math.pi:
                theta_to_goal += (2*math.pi)
            else:
                pass
            twist_msg.angular.z = self.__p_controller_coefficient * theta_to_goal
        else:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.__call_catch_turtle_service(self.__turtle_to_catch.name)
            self.__turtle_to_catch = None
        self.__cmd_vel_publisher.publish(twist_msg)

    def __call_catch_turtle_service(self, turtle_name: str):
        client = self.create_client(
            srv_type=CatchTurtle,
            srv_name="catch_turtle"
        )

        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for \"catch_turtle\" service...")

        request = CatchTurtle.Request()
        request.name = turtle_name
        future = client.call_async(request)
        future.add_done_callback(
            partial(self.__callback_call_catch_turtle, turtle_name=turtle_name)
        )

    def __callback_call_catch_turtle(self, future: Future, turtle_name: str):
        response: CatchTurtle.Response
        try:
            response = future.result() # is None
            if not response.success:
                self.get_logger().error(f"Turtle {turtle_name} could not be caught")
        except Exception as error:
            self.get_logger().error(f"Catch turtle service call failed {error}")


def main(args=None):
    rclpy.init(args=args)
    try:
        catch_turtle_node = CatchTheTurtle()
        rclpy.spin(catch_turtle_node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()