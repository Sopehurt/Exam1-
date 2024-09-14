
# Turtle Fun Project

## Overview

This project is part of a ROS2-based simulation using the `turtlesim` package. It includes a custom teleoperation node (`teleop_key.py`) that allows users to control the turtle's movement using keyboard inputs. This project provides a simple, interactive way to explore ROS2 by navigating a virtual turtle within a 2D environment.

## Prerequisites

Before running this project, make sure you have the following prerequisites installed and configured on your system:

1. **ROS2 (Humble or later)** installed and sourced correctly.
2. **Python 3.x** for executing Python-based ROS2 nodes.
3. **colcon** build tool installed, typically included with ROS2.
4. **Turtlesim** package, which is included with ROS2 by default.

If you do not have ROS2 installed, follow the [ROS2 Installation Instructions](https://docs.ros.org/en/humble/Installation.html) specific to your operating system.

## Getting Started

Follow these steps to clone, build, and run the `turtle_fun` project:

### Step 1: Clone the Repository

First, open your terminal and clone this repository into your ROS2 workspace (usually located in `~/ros2_ws`):

```bash
git clone https://github.com/Sopehurt/Exam1-.git
```

Change to the directory where the repository was cloned:

```bash
cd Exam1-
```

### Step 2: Build the Workspace

To compile the ROS2 package, use the following command:

```bash
colcon build --packages-select turtle_fun
```

This command builds the `turtle_fun` package and its dependencies, preparing it for use in ROS2. Ensure there are no build errors before proceeding.

### Step 3: Source the Workspace

After the build process completes, source the workspace to overlay the environment with the newly built packages:

```bash
source install/setup.bash
```

This step is critical because it allows the terminal to recognize the `turtle_fun` package and its associated nodes.

### Step 4: Install Any Additional Dependencies

Ensure that all required ROS2 packages, such as `turtlesim`, are installed by running:

```bash
sudo apt update
sudo apt install ros-humble-turtlesim
```

## Usage

### 1. Launch the Turtle Simulation

Once the workspace is built and sourced, you can start the turtle simulation. In a new terminal (make sure to source your ROS2 installation), run the following command:

```bash
ros2 launch turtle_fun turtle_fun.launch.py
```

This will launch the `turtlesim` environment, and you'll see the turtle in a window, ready to be controlled.

### 2. Teleoperate the Turtle

To teleoperate the turtle using the keyboard, open another terminal (also sourced), and run the following command:

```bash
ros2 run turtle_fun teleop_key.py
```

You can now use the keyboard to control the turtle’s movements:

#### Keyboard Controls:

- `w`: Move forward
- `a`: Turn left
- `s`: Move backward
- `d`: Turn right
- `r`: Reset
- `i`: Spawn pizza
- `o`: Save
- `p`: Clear

Press the respective keys to move the turtle in the desired direction.

### 3. Stopping the Simulation

To stop both the simulation and the teleop node, press `Ctrl+C` in each terminal where they are running.

## Troubleshooting

1. **Build Errors**: If you encounter errors during the `colcon build` process, ensure all ROS2 dependencies are installed and that you are in the correct workspace.
   
2. **Unable to Control Turtle**: If the turtle does not move when using the `teleop_key.py` node, ensure that both the simulation and teleop nodes are running and that your terminal is focused on the window where `teleop_key.py` is active.

3. **Workspace Not Found**: Always make sure you have sourced the workspace (`source install/setup.bash`) before running any ROS2 commands.

## Customization

You can extend the functionality of this project by adding new nodes, services, or actions. The `turtlesim` package allows for spawning multiple turtles, changing pen colors, or controlling turtles using more advanced logic.

To learn more, consult the [ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html).

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
