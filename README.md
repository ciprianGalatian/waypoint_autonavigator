# waypoint_autonavigator

**waypoint_autonavigator** is a ROS 2 package designed for waypoint-based autonomous navigation using the ROS 2 Navigation Stack (Nav2). It enables a robot to follow a predefined set of waypoints autonomously.

## Features

- Integration with the ROS 2 Navigation Stack (Nav2)
- Launch-ready waypoint navigation system
- Modular and extensible design

## Requirements

- ROS 2 (recommended: Humble)
- `colcon` build system
- A functioning ROS 2 workspace (e.g., `~/ros2_ws`)
- `nav2` packages installed and configured

## Installation

### 1. Clone the Repository

Open a terminal and navigate to your ROS 2 workspace `src` directory:

```bash
cd ~/ros2_ws/src
git clone https://github.com/ciprianGalatian/waypoint_autonavigator.git

2. Build the Workspace
Once cloned, go to the root of your workspace and build:

cd ~/ros2_ws
colcon build
