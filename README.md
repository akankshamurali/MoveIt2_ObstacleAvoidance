PROJECT STRUCTURE

launch/ – Contains launch files to start the simulation:
  plan_around_box.launch.py – Runs the demo in loop mode, with the Panda arm moving between fixed start and goal positions while avoiding a cube.
  plan_around_box_interactive.launch.py – Lets you set the goal position interactively in RViz, while keeping the cube as an obstacle.

src/ – C++ source code for the motion planning node. Handles robot setup, obstacle creation, planning, and execution.
  plan_around_box.cpp - Main C++ node implementing:    
    Loading the Panda robot description.    
    Connecting to the MoveGroupInterface for motion planning.    
    Adding a static cube obstacle to the Planning Scene.    
    Planning collision-free trajectories from start to goal.    
    Executing the planned motion.    
    Looping (in auto mode) or responding to user input (in interactive mode).    
    Publishing obstacle information for RViz visualization.

CMakeLists.txt – Build configuration for compiling the package with colcon build.

package.xml – ROS 2 package manifest with dependencies and metadata.

HOW TO RUN THE SIMULATION

1. Prerequisites
Make sure you are running Ubuntu 22.04 with ROS 2 Humble installed. You’ll also need MoveIt 2 packages:
    sudo apt update
    sudo apt install -y \
      ros-humble-moveit \
      ros-humble-moveit-ros-planning-interface \
      ros-humble-moveit-resources-panda-moveit-config \
      ros-humble-rviz2

2. Create a directory and Clone the Repository
   
    mkdir -p ~/ws_panda_box/src
   
    cd ~/ws_panda_box/src
   
    git clone https://github.com/akankshamurali/MoveIt2_ObstacleAvoidance.git

4. Build the Package
    cd ~/ws_panda_box
    colcon build --merge-install
    source install/setup.bash

Why --merge-install?
By default, colcon build installs each package into its own folder inside install/. Using --merge-install combines all packages into a single unified install/ directory.
This makes it easier for small workspaces like this one because you only need to source one install/setup.bash.

4. Running the Simulation
Option A — Interactive Mode (Set Goal in RViz)
Allows you to manually set the target pose in RViz while the robot avoids the cube.
    ros2 launch path_planning_minimal plan_around_box_interactive.launch.py
Move the 6-DoF goal marker to your desired target pose.
Click Plan, then Execute.

Option B — Looping Demo
    ros2 launch path_planning_minimal plan_around_box.launch.py



