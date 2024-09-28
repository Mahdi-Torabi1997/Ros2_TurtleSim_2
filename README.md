## Overview

This project demonstrates the use of ROS 2 and the Turtlesim simulation to create a simple "catch the turtle" game using a custom controller. The setup consists of two nodes: one node spawns turtles at random locations in the Turtlesim environment, while another node controls a turtle that attempts to catch the spawned turtles.

### Key Components

1. **Turtle Spawner**:
   - Spawns new turtles at random locations using the `turtlesim` simulation.
   - Publishes the positions of the alive turtles.
   - Provides a service for removing (catching) a turtle from the environment.

2. **Turtle Controller**:
   - Subscribes to the current pose of the controlled turtle (`turtle1`) and the list of alive turtles.
   - Based on a ROS parameter, it either catches the closest turtle or simply targets the first turtle in the list.
   - Moves the turtle towards its target and attempts to catch it by calling the catch service.

### Technologies

- **ROS 2**: Utilized for communication between nodes and managing services.
- **Turtlesim**: The 2D simulation environment used to visualize and interact with the turtles.
- **Python**: Used to implement both the turtle spawner and controller.

This project highlights basic ROS 2 concepts like publishers, subscribers, services, and parameter handling, alongside a fun interactive simulation with Turtlesim.
