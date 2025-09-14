# VFH for Collision Avoidance
![Screenshot 2024-11-02 163145](https://github.com/user-attachments/assets/8888b4bd-2611-4a3a-a0bb-dc32d3a21273)
[![Watch the video](https://img.youtube.com/vi/VteC_HYPnDo/maxresdefault.jpg)](https://youtu.be/VteC_HYPnDo)

### [3D simulation](https://youtu.be/VteC_HYPnDo)
# Robot Navigation Simulation with Pure Pursuit and Vector Field Histogram

This project simulates a robot navigating towards a target while avoiding dynamic obstacles using the **Pure Pursuit** Method for path following and **Vector Field Histogram (VFH)** for obstacle avoidance.

---

## Overview

This simulation demonstrates:

1. **Pure Pursuit Path Following**  
   The robot calculates the angle toward the target and adjusts its orientation accordingly to follow the path.

2. **Vector Field Histogram (VFH) Obstacle Avoidance**  
   The robot scans its environment to detect nearby obstacles, generating a histogram of distances within its field of view. When obstacles are detected within a certain threshold distance, the robot adjusts its orientation to avoid them.

3. **Dynamic Obstacles**  
   Obstacles move randomly, adding complexity to the robot's navigation task.

---

## Features

- **Dynamic Targeting**: The robot continuously re-orients toward a target position.
- **Obstacle Avoidance**: Upon detecting obstacles, the robot selects a safe angle to prevent collisions.
- **Real-time Plotting**: The simulation visually represents the robot, target, obstacles, and sensor beams.
- **Polar Histogram Display**: An inset plot shows the polar histogram of obstacle distances.

---

