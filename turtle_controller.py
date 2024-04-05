#!/usr/bin/env python3
import math
import rclpy
from functools import partial
from rclpy.node import Node

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle


class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")  # Initialize the Node with the name "turtle_controller"
        # Declare and get a ROS parameter to decide catching strategy
        self.declare_parameter("catch_closest_turtle_first", True)
        self.catch_closest_turtle_first_ = self.get_parameter("catch_closest_turtle_first").value
        
        # Initialize member variables
        self.turtle_to_catch_ = None  # Target turtle to catch
        self.pose_ = None  # Current pose of the turtle being controlled
        
        # Publisher to control turtle's movement by publishing Twist messages
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        # Subscriber to turtle's current pose
        self.pose_subscriber_ = self.create_subscription(Pose, "turtle1/pose", self.callback_turtle_pose, 10)
        # Subscriber to get the list of alive turtles
        self.alive_turtles_subscriber_ = self.create_subscription(TurtleArray, "alive_turtles", self.callback_alive_turtles, 10)
        # Timer for the control loop, set to execute every 0.01 seconds
        self.control_loop_timer_ = self.create_timer(0.01, self.control_loop)

    def callback_turtle_pose(self, msg):
        # Callback for updating the current pose of the turtle
        self.pose_ = msg

    def callback_alive_turtles(self, msg):
        # Callback for determining the next target turtle based on the alive turtles' list
        if len(msg.turtles) > 0:  # Check if there are any alive turtles
            if self.catch_closest_turtle_first_:
                # Find the closest turtle if the parameter is True
                closest_turtle = None
                closest_turtle_distance = None
                for turtle in msg.turtles:
                    # Calculate distance to each turtle
                    dist_x = turtle.x - self.pose_.x
                    dist_y = turtle.y - self.pose_.y
                    distance = math.sqrt(dist_x**2 + dist_y**2)
                    # Update closest turtle if necessary
                    if closest_turtle is None or distance < closest_turtle_distance:
                        closest_turtle = turtle
                        closest_turtle_distance = distance
                self.turtle_to_catch_ = closest_turtle
            else:
                # Otherwise, target the first turtle in the list
                self.turtle_to_catch_ = msg.turtles[0]

    def control_loop(self):
        # Main control loop for moving towards the target turtle
        if self.pose_ is None or self.turtle_to_catch_ is None:
            # Do nothing if we don't have current pose or a target
            return

        # Calculate distance and direction to the target turtle
        dist_x = self.turtle_to_catch_.x - self.pose_.x
        dist_y = self.turtle_to_catch_.y - self.pose_.y
        distance = math.sqrt(dist_x**2 + dist_y**2)

        msg = Twist()  # Initialize a Twist message for velocity command

        if distance > 0.5:
            # Move towards the turtle if it's not yet caught
            msg.linear.x = 2 * distance  # Control linear speed based on distance
            goal_theta = math.atan2(dist_y, dist_x)  # Desired orientation
            diff = goal_theta - self.pose_.theta  # Calculate orientation difference
            # Adjust for the shortest rotation direction
            if diff > math.pi:
                diff -= 2 * math.pi
            elif diff < -math.pi:
                diff += 2 * math.pi
            msg.angular.z = 6 * diff  # Control angular speed based on orientation difference
        else:
            # Target reached, stop moving and attempt to catch the turtle
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.call_catch_turtle_server(self.turtle_to_catch_.name)
            self.turtle_to_catch_ = None  # Reset the target turtle after attempting to catch

        self.cmd_vel_publisher_.publish(msg)  # Publish the velocity command

    def call_catch_turtle_server(self, turtle_name):
        # Create a client for the CatchTurtle service and call it to catch the turtle
        client = self.create_client(CatchTurtle, "catch_turtle")
        while not client.wait_for_service(1.0):
            # Wait for the service to become available
            self.get_logger().warn("Waiting for Server...")

        request = CatchTurtle.Request()
        request.name = turtle_name  # Set the target turtle name in the request

        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_catch_turtle, turtle_name=turtle_name))

    def callback_call_catch_turtle(self, future, turtle_name):
        # Callback for processing the response from the CatchTurtle service
        try:
            response = future.result()
            if not response.success:
                # Log an error if the turtle couldn't be caught
                self.get_logger().error(f"Turtle {turtle_name} could not be caught")
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

# Entry point of the script
def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS 2 Python client library
    node = TurtleControllerNode()  # Create an instance of the TurtleControllerNode
    rclpy.spin(node)  # Keep the node running
    rclpy.shutdown()  # Shutdown the ROS 2 Python client library when done

if __name__ == "__main__":
    main()

if __name__ == "__main__":
    main()
