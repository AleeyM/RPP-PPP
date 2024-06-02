# RPP-PPP
R&amp;RPP-RRR Robotic Arm Simulation

Overview
This project provides a simulation of a 6-DOF robotic arm with an RPPRRR configuration using ROS2, MoveIt2, and Gazebo. The steps below will guide you through cloning the repository, building the workspace, and launching the simulation.

Instructions to Run the Simulation
Follow these steps to set up and run the simulation:

Step 1: Open Terminal
Open your terminal application.

Step 2: Clone the Repository
Run the following command to clone the project repository:

git clone https://github.com/AleeyM/RPP-PPP/

Step 3: Navigate to the Workspace
Change the directory to the workspace:

cd test_ws

Step 4: Build the Workspace
Build the workspace using Colcon:

colcon build
Step 5: Source the Setup Script
Source the setup script to overlay this workspace on your ROS2 environment:


source install/setup.bash

Step 6: Launch the Simulation
Launch the simulation using the provided launch file:

ros2 launch RPP_moveit_config demo.launch.py


Conclusion
By following these steps, you will set up and run the simulation of the 6-DOF robotic arm. This simulation utilizes ROS2, MoveIt2, and Gazebo to provide a comprehensive environment for motion planning and execution.
