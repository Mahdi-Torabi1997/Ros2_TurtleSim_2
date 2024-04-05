#!/usr/bin/env python3
from functools import partial
import random
import math
import rclpy
from rclpy.node import Node

from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle


class TurtleSpawner(Node):
    def __init__(self):
        super().__init__("turtle_spawner")  # Initialize the Node with the name "turtle_spawner"
        # Declare and get parameters for spawning frequency and turtle name prefix
        self.declare_parameter("spawn_frequency", 1.0)
        self.spawn_frequency_ = self.get_parameter("spawn_frequency").value
        self.declare_parameter("turtle_name_prefix", "turtle")
        self.turtle_name_prefix_ = self.get_parameter("turtle_name_prefix").value
        
        self.turtle_counter_ = 0  # Counter to keep track of spawned turtles
        self.alive_turtles_ = []  # List to keep track of alive turtles
        # Publisher for alive turtles
        self.alive_turtles_publisher_ = self.create_publisher(TurtleArray, "alive_turtles", 10)
        # Timer for spawning new turtles at the specified frequency
        self.spawn_turtle_timer_ = self.create_timer(1.0/self.spawn_frequency_, self.spawn_new_turtle)
        # Service for catching turtles
        self.catch_turtle_service_ = self.create_service(CatchTurtle, "catch_turtle", self.callback_catch_turtle)

    # Service callback to catch (kill) a turtle by name
    def callback_catch_turtle(self, request, response):
        self.call_kill_server(request.name)  # Call the kill service for turtlesim
        response.success = True
        return response

    # Publish the list of alive turtles
    def publish_alive_turtles(self):
        msg = TurtleArray()
        msg.turtles = self.alive_turtles_
        self.alive_turtles_publisher_.publish(msg)

    # Timer callback to spawn a new turtle
    def spawn_new_turtle(self):
        self.turtle_counter_ += 1
        name = f"{self.turtle_name_prefix_}{self.turtle_counter_}"
        x = random.uniform(0.0, 11.0)  # Random x position
        y = random.uniform(0.0, 11.0)  # Random y position
        theta = random.uniform(0.0, 2*math.pi)  # Random orientation
        self.call_spawn_server(name, x, y, theta)  # Call the spawn service

    # Client call to spawn a new turtle in turtlesim
    def call_spawn_server(self, turtle_name, x, y, theta):
        client = self.create_client(Spawn, "spawn")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server...")

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = turtle_name

        future = client.call_async(request)
        # Use partial to pass extra arguments to the callback
        future.add_done_callback(partial(self.callback_call_spawn, turtle_name=turtle_name, x=x, y=y, theta=theta))

    # Callback for handling the response from the spawn service
    def callback_call_spawn(self, future, turtle_name, x, y, theta):
        try:
            response = future.result()
            if response.name != "":
                self.get_logger().info(f"Turtle {response.name} is now alive")
                # Add the newly spawned turtle to the list of alive turtles
                new_turtle = Turtle()
                new_turtle.name = response.name
                new_turtle.x = x
                new_turtle.y = y
                new_turtle.theta = theta
                self.alive_turtles_.append(new_turtle)
                self.publish_alive_turtles()  # Update the list of alive turtles
        except Exception as e:
            self.get_logger().error(f"Service call failed {e}")

    # Client call to kill a turtle by name in turtlesim
    def call_kill_server(self, turtle_name):
        client = self.create_client(Kill, "kill")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server...")

        request = Kill.Request()
        request.name = turtle_name

        future = client.call_async(request)
        # Use partial to pass extra arguments to the callback
        future.add_done_callback(partial(self.callback_call_kill, turtle_name=turtle_name))

    # Callback for handling the response from the kill service
    def callback_call_kill(self, future, turtle_name):
        try:
            future.result()
            # Remove the turtle from the list of alive turtles
            self.alive_turtles_ = [turtle for turtle in self.alive_turtles_ if turtle.name != turtle_name]
            self.publish_alive_turtles()  # Update the list of alive turtles
        except Exception as e:
            self.get_logger().error(f"Service call failed {e}")

# Entry point of the script
def main(args=None):
    rclpy.init(args=args)  # Initialize ROS 2 Python client library
    node = TurtleSpawner()  # Create an instance of TurtleSpawner Node
    rclpy.spin(node)  # Keep the node running
    rclpy.shutdown()  # Shutdown ROS 2 Python client library when done

if __name__ == "__main__":
    main()
