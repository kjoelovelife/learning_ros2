#!/usr/bin/env python3

import math
import random
import rclpy

from asyncio import Future
from functools import partial
from rclpy.node import Node
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from lesson_interfaces.msg import Turtle
from lesson_interfaces.msg import TurtleArray
from lesson_interfaces.srv import CatchTurtle

class TurtleSpawner(Node):
    def __init__(self, node_name: str="turtle_spawner_node"):
        super().__init__(node_name)
        self.__turtlesim_bound = 11.0
        self.__turtle_counter = 0
        self.__turtle_name_prefix = "turtle"
        self.__alive_turtles = []


        # Publisher
        self.__alive_turtles_publisher = self.create_publisher(
            msg_type=TurtleArray,
            topic="alive_turtles",
            qos_profile=10
        )

        # Service
        self.__catch_turtle_service = self.create_service(
            srv_type=CatchTurtle,
            srv_name="catch_turtle",
            callback=self.__callback_catch_turtle
        )

        # Timer
        self.__spawn_turtle_timer = self.create_timer(
            timer_period_sec=2.0,
            callback=self.__callback_spawn_turtle_timer
        )



    def __callback_catch_turtle(
        self, 
        request: CatchTurtle.Request, 
        response: CatchTurtle.Response
    ) -> CatchTurtle.Response:
        self.__call_kill_service(request.name)
        response.success = True
        return response

    def __call_kill_service(self, turtle_name: str):
        client = self.create_client(
            srv_type=Kill, 
            srv_name="kill"
        )
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for \"/kill\" service...")

        request = Kill.Request()
        request.name = turtle_name
        future = client.call_async(request)
        future.add_done_callback(
            partial(self.__callback_call_kill_service, turtle_name=turtle_name)
        )

    def __callback_call_kill_service(self, future: Future, turtle_name: str):
        try:
            response = future.result() # is None
            turtle: Turtle
            for index, turtle in enumerate(self.__alive_turtles):
                if turtle.name == turtle_name:
                    del self.__alive_turtles[index]
                    self.__publish_alive_turtles()
                    break
        except Exception as error:
            self.get_logger().error(f"Service call failed, {error}")

    def __publish_alive_turtles(self):
        msg = TurtleArray()
        msg.turtle_array = self.__alive_turtles
        self.__alive_turtles_publisher.publish(msg)

    def __callback_spawn_turtle_timer(self):
        self.__turtle_counter += 1
        name = f"{self.__turtle_name_prefix}{self.__turtle_counter}"
        x = random.uniform(0.0, self.__turtlesim_bound)
        y = random.uniform(0.0, self.__turtlesim_bound)
        theta = random.uniform(0.0, 2*math.pi)
        self.__call_spawn_service(name, x, y, theta)

    def __call_spawn_service(self, turtle_name: str, x: float, y: float, theta: float):
        client = self.create_client(
            srv_type=Spawn,
            srv_name="spawn"
        )
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for \"/spawn\" service...")

        request = Spawn.Request()
        request.x = x
        request.y = y 
        request.theta = theta
        request.name = turtle_name
        future = client.call_async(request)
        future.add_done_callback(
            partial(self.__callback_call_spawn_service, turtle_name=turtle_name, x=x, y=y, theta=theta)
        )

    def __callback_call_spawn_service(
        self, 
        future: Future,
        turtle_name: str, 
        x: float, 
        y: float, 
        theta: float
    ):
        response: Spawn.Response
        try: 
            response = future.result()
            if response.name != "":
                self.get_logger().info(f"Turtle {response.name} is now alive")
                new_turtle = Turtle()
                new_turtle.name = response.name
                new_turtle.pose.x = x
                new_turtle.pose.y = y
                new_turtle.pose.theta = theta
                self.__alive_turtles.append(new_turtle)
                self.__publish_alive_turtles()
        except Exception as error:
            self.get_logger().error(f"Service call failed {error}")

def main(args=None):
    rclpy.init(args=args)
    try:
        node = TurtleSpawner()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()