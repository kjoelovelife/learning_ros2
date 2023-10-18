#!/usr/bin/env Python3

import math
import random
import rclpy

from asyncio import Future
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
from geometry_msgs.msg import Twist

class FollowTheTurtle(Node):
    def __init__(self, node_name: str="follow_the_turtle_node"):
        super().__init__(node_name)

        self.__turtle1_pose = None
        self.__turtle2_pose = None
        self.__trutle2_name = "turtle2"
        self.__turtlesim_bound = 11.0
        self.__p_controller_coefficient = 1.0

        self.__turtle1_pose_subscriber = self.create_subscription(
            msg_type=Pose,
            topic="/turtle1/pose",
            qos_profile=10,
            callback=self.__callback_turtle1_pose
        )

        self.__turtle2_pose_subscriber = self.create_subscription(
            msg_type=Pose,
            topic=f"/{self.__trutle2_name}/pose",
            qos_profile=10,
            callback=self.__callback_turtle2_pose
        )

        self.__turtle2_cmd_vel_publisher = self.create_publisher(
            msg_type=Twist,
            topic=f"/{self.__trutle2_name}/cmd_vel",
            qos_profile=10
        )

        self.__control_loop = self.create_timer(
            timer_period_sec=0.01,
            callback=self.__callback_control_loop
        )

        self.__call_turtlesim_spawn_service()

    def __callback_turtle1_pose(self, msg: Pose):
        self.__turtle1_pose = msg

    def __callback_turtle2_pose(self, msg: Pose):
        self.__turtle2_pose = msg


    def __call_turtlesim_spawn_service(self):
        client = self.create_client(
            srv_type=Spawn,
            srv_name="spawn"
        )

        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for the \"/Spawn\" service...")

        request = Spawn.Request()
        request.name = self.__trutle2_name
        request.x = random.uniform(0.0, self.__turtlesim_bound)
        request.y = random.uniform(0.0, self.__turtlesim_bound)
        request.theta = random.uniform(0.0, 2*math.pi)
        future = client.call_async(request)
        future.add_done_callback(self.__callback_call_turtlesim_spawn_service)


    def __callback_call_turtlesim_spawn_service(self, future: Future):
        response: Spawn.Response
        try:
            response = future.result()
            self.get_logger().info(f"Turtle {response.name} spawned successfully!")
        except Exception as error:
            self.get_logger().error(f"Turtle spawn failed. {error}")
        
    def __callback_control_loop(self):
        if self.__turtle1_pose == None or self.__turtle2_pose == None:
            return
        self.__turtle1_pose: Pose
        self.__turtle2_pose: Pose
        distance_x = self.__turtle1_pose.x - self.__turtle2_pose.x
        distance_y = self.__turtle1_pose.y - self.__turtle2_pose.y
        distance = math.sqrt(math.pow(distance_x, 2)+math.pow(distance_y, 2))

        twist_msg = Twist()

        # Use P controller
        if distance > 0.5:
            twist_msg.linear.x = self.__p_controller_coefficient * 0.5 * distance
            goal_theta = math.atan2(distance_y, distance_x)
            theta_to_goal = goal_theta - self.__turtle2_pose.theta
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

        self.__turtle2_cmd_vel_publisher.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        follow_the_turtle_node = FollowTheTurtle()
        rclpy.spin(follow_the_turtle_node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()