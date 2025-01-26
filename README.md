# Project-ROBO5  
**Mapping and Navigation on Turtlebot3**  

## Overview  
This project uses the Turtlebot3 platform to implement features such as motion control, mapping, and trajectory tracking. Using ROS1 Noetic and MATLAB, we worked to develop a semi-autonomous robotic system capable of navigating its environment while avoiding obstacles and creating maps. Key components include SLAM (Simultaneous Localization and Mapping), teleoperation, and the application of an Extended Kalman Filter for trajectory tracking.


## Repository Structure  

### Files Included in the Repository 
- **`Kalman_all.m`**: MATLAB script used for processing sensor data (IMU and Odometry) with a Kalman filter.  
- **`catkin_ws_computer.zip`**: A zipped Catkin workspace containing ROS1 packages and configurations used for the project. Extract this folder to work with the ROS environment.  
- **`command_turtle.txt`**: This file was used as a reference for frequently run commands like starting nodes, controlling the Turtlebot, and managing the ROS environment. 
- **`simulation.m`**: MATLAB script for running simulations based on Turtlebot3 sensor data gathered with "Kalman_all.m". 

---

## Key Features  

### 1. **Motion Control**  
- **OpenCR Test Buttons:** Initial testing of forward movement and rotation (30 cm forward, 180° rotation).  
- **Teleoperation Node:** Controlled movement remotely via keyboard. Limited key customization (QWERTY layout: W, A, S, D).  
- **Challenges:** Attempted but failed to integrate a DualShock4 controller due to lack of a Bluetooth sensor.

### 2. **Mapping (SLAM)**  
- **SLAM Node:** Created maps using the Turtlebot3’s LiDAR sensor.  
- **Rviz Integration:** Real-time visualization of point cloud data for mapping.  
- **Obstacle Detection:** Static obstacles were detected automatically, but the system occasionally misidentified humans as fixed obstacles.  
- **Enhancements:** A camera could improve mapping accuracy and add background information to the maps.  

### 3. **Semi-Autonomous Navigation**  
- After mapping, the robot navigates between two points on the map.  
- **Obstacle Avoidance:** Real-time detection and bypassing of new obstacles using LiDAR.  
- **Dynamic Path Planning:** Recalculates the optimal path when new obstacles are detected.  

### 4. **Trajectory Tracking**  
- **Kalman Filter:** Implemented for accurate estimation of position, velocity, and orientation.  
    - Applied to both simulation data (Gazebo, MATLAB Simulink) and real-world robot tests.  
- **MATLAB Integration:**  
    - Simulated trajectories using a Simulink robot model.  
    - Collected real-time sensor data from the Turtlebot for Kalman filtering.  

---

## Results  
- Successfully integrated SLAM for mapping and semi-autonomous navigation.  
- Developed a Kalman Filter in MATLAB for trajectory tracking using simulated and real-world data.  
- Created a foundation for future enhancements, such as live plotting, a user-friendly interface, and "return to base" functionality.

---

## Remaining Tasks  
- Test the Kalman Filter fully on the real Turtlebot.  
- Develop a graphical user interface for easier navigation and control.  
- Implement advanced motion planning for unexplored environments.  
