Turtle Fun Project
Overview
This project is part of a ROS2-based simulation using the turtlesim package. It includes a custom teleoperation node (teleop_key.py) that allows the user to control the turtle's movement using keyboard inputs.

Prerequisites
Before running this project, ensure you have the following installed:

ROS2 Humble (or later) installed and sourced correctly.
Python 3.x.
colcon build tool installed via ROS2.
If ROS2 is not installed, you can follow the installation guide for your system: ROS2 Installation Instructions.

Getting Started
Step 1: Clone the Repository
First, you need to clone this repository into your workspace:
git clone https://github.com/Sopehurt/Exam1-.git
Then, change into the workspace directory:
cd Exam1-
Step 2: Build the Workspace
Once inside the workspace, you will need to build the package using colcon:
colcon build

This command compiles the turtle_fun package and prepares it for use.

Step 3: Source the Workspace
After the build is complete, source your workspace to overlay your current environment with the newly built packages:
source install/setup.bash
This step is essential so that the ROS2 environment knows about your newly built packages.

Usage
Launch the Turtle Simulation
To start the turtle simulation, run the following command:
ros2 launch turtle_fun turtle_fun.launch.py
This will launch the turtle simulation, allowing you to control the turtle.

Teleoperate the Turtle
With the simulation running, you can control the turtle using the teleop node:
ros2 run turtle_fun teleop_key.py
Keyboard Controls:
W: Move forward
A: Turn left
S: Move backward
D: Turn right
Press the corresponding keys to move the turtle in the desired direction.

Stopping the Simulation
To stop the simulation, use Ctrl+C in the terminal where the simulation was launched.

License
This project is licensed under the MIT License - see the LICENSE file for details.
This README provides detailed instructions from cloning the repository to building and using the package. Let me know if you'd like to add or adjust any specific details!
