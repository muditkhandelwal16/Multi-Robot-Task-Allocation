# Multi-Robot Task Allocation for TurtleBot3

This repository contains a ROS 2 workspace for simulating **multi-robot task allocation** using multiple TurtleBot3 robots in a custom warehouse environment.

The system brings together:

- A custom **warehouse Gazebo world** with up to five TurtleBot3 Burger robots.
- **Global path planning** on an occupancy grid (A\* with inflation costmap).
- **Task allocation** using a cost-based matching (Hungarian algorithm) between robots and tasks.
- **Localization** using AMCL on a static map.
- A simple **PD motion controller** to track the planned paths.
- A unified **bringup launch file** (`dta_bringup/bringup.launch.py`) that starts the full pipeline.

The goal is to demonstrate a complete multi-robot workflow from world setup → localization → task allocation → path planning → motion execution for multiple robots.

---

## Features

- **Warehouse simulation**

  - Custom `warehouse.world` with static obstacles.
  - Up to five individual TurtleBot3 Burger models with unique names.

- **Path planning & costmap**

  - 2D occupancy grid-based planning.
  - A\* implementation in Python.
  - Inflated costmap via `inflation_costmap.yaml` to keep paths away from obstacles.

- **Task allocation**

  - Tasks are represented as goal positions in the environment.
  - A cost matrix is built from (robot, task) pairs using path-length or distance estimates.
  - **Hungarian algorithm** used to compute a minimum-cost assignment.

- **Localization**

  - AMCL configuration and launch files for multi-robot localization on a static map.
  - Support for map server and AMCL per robot.

- **Motion planning / control**

  - PD controller to follow waypoints from the planner.
  - Multi-robot launch setup so each robot tracks its assigned trajectory.

- **Custom interfaces**
  - `PlanPath.srv` service in `custom_interfaces` for path planning requests from other nodes.

> Note: Upstream TurtleBot3 packages (`turtlebot3`, `turtlebot3_msgs`, `turtlebot3_simulations`, etc.) are **not** included in this repository. They are installed as dependencies from ROS packages.

---

## Prerequisites

- Ubuntu with a working **ROS 2** installation (e.g., Humble, Foxy, etc.).
- Gazebo with `gazebo_ros` integration.
- TurtleBot3 ROS 2 packages.
- Python 3 with the packages listed in `requirements.txt`.

Example (replace `<ros-distro>` with your ROS 2 distribution name, e.g., `humble`, `foxy`, etc.):

    sudo apt update
    sudo apt install \
      ros-<ros-distro>-desktop \
      ros-<ros-distro>-gazebo-ros-pkgs \
      ros-<ros-distro>-turtlebot3*

Set the TurtleBot3 model (if not already done):

    echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
    source ~/.bashrc

---

## Installation

Clone this repository as a ROS 2 workspace:

    cd ~
    git clone git@github.com:muditkhandelwal16/Multi-Robot-Task-Allocation.git turtlebot3_ws
    cd turtlebot3_ws

(Optional but recommended) Create and activate a Python virtual environment:

    python3 -m venv venv
    source venv/bin/activate

Install Python dependencies:

    pip install --upgrade pip
    pip install -r requirements.txt

Build the workspace with `colcon`:

    source /opt/ros/<ros-distro>/setup.bash
    cd ~/turtlebot3_ws
    colcon build
    source install/setup.bash

You may want to add the workspace overlay to your `.bashrc`:

    echo "source ~/turtlebot3_ws/install/setup.bash" >> ~/.bashrc

---

## Running the System

After building, the entire system (Gazebo world, robot spawning, localization, task allocation, path planning, and motion control) is started using a **single bringup launch file**.

Open a terminal:

    cd ~/turtlebot3_ws

    # Source ROS 2
    source /opt/ros/<ros-distro>/setup.bash

    # Source this workspace (if not already in your .bashrc)
    source install/setup.bash

    # Run the full multi-robot bringup
    ros2 launch dta_bringup bringup.launch.py

This launch file is expected to:

- Start Gazebo with the custom warehouse world.
- Spawn multiple TurtleBot3 robots in the world.
- Start map server + AMCL for localization.
- Run the path planning and task allocation nodes.
- Launch the PD motion controllers so each robot executes its assigned path.

(If you adjust the launch file behavior, update this section to match.)

---

## Areas for Improvement / Future Work

This project is a working prototype, and there are several important directions for improvement:

### 1. Collision avoidance between TurtleBots

- Currently, each robot follows its planned path independently using the PD motion controller.
- Paths are generated considering **static obstacles** via the inflation costmap, but **inter-robot collisions are not explicitly handled**.

Potential improvements:

- Integrate a local planner or velocity-obstacle / ORCA-style collision avoidance.
- Treat other robots as dynamic obstacles in the costmap (dynamic layers).
- Add centralized or distributed conflict resolution for crossing / overlapping paths.

### 2. Smarter and more scalable task allocation

- Right now, the task allocation builds a cost matrix using **path costs between all robots and all tasks**.
- Computing full A\* paths for every (robot, task) pair does not scale well as the number of robots and tasks increases.

Potential improvements:

- Use heuristic distance estimates (Euclidean / Manhattan) as a first pass instead of full A\* for every pair.
- Cache or incrementally update path costs rather than recomputing from scratch each time.
- Explore more advanced / distributed task allocation algorithms (e.g., CBBA variants, market/auction-based methods) that do not require computing every possible path explicitly.

---

## Notes

- TurtleBot3 core packages are treated as external dependencies and are **not** version-controlled in this repo.
- This workspace is primarily intended as a research / learning platform for multi-robot task allocation on TurtleBot3-like systems, rather than a production-ready navigation stack.

---

## License

Add your preferred license here (e.g., MIT, BSD, Apache 2.0) once you decide how you want to license this work.
