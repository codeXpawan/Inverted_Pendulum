# Inverted Pendulum - ROS Project

This is a **ROS-based simulation** of an inverted pendulum mounted on a cart, a classic control systems problem.  
The goal is to balance the pendulum in the upright position using control strategies like **PID**, while simulating the system in **RViz** and **Gazebo**.
Simulation works perfectly while physical model fail due to ineffecient capacity of Motor.

---

## 📌 Project Overview

- Modeled the pendulum-cart system in URDF
- Added joint dynamics and constraints
- Visualized in RViz
- Simulated in Gazebo
---

## 🛠️ Technologies Used

- ROS2 Humble
- Gazebo — for simulation
- RViz — for visualization
- URDF / Xacro — for robot description
- microros for esp32 connection
---

## To Run Simulation
```bash
#clone the repo
git clone https://github.com/codeXpawan/Inverted_Pendulum.git
cd Inverted_Pendulum/ROS
colcon build
source install/setup.bash
#To launch run simulation
ros2 launch robot_description gazebo_simulation.launch.py
ros2 run robot_control robot_control
```
