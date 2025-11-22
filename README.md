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

## Kinematics Control


https://github.com/user-attachments/assets/930dcdc1-376b-4607-ac06-9ad451f5b8b5

---

 ## Kinematics*  
   Model of a differential-drive mobile robot, using wheel angular velocities, body velocities, and pose representation

---
<img width="842" height="589" alt="Screenshot 2025-11-22 092352" src="https://github.com/user-attachments/assets/2aaef595-81af-428b-a4be-8c81d8815dc5" />


---

 ## Robot Pose Representation**  
   Robot pose at time step _i_ is:
   
   <img width="143" height="28" alt="image" src="https://github.com/user-attachments/assets/2a1eb356-1b6d-4c57-9e4e-8f205c809999" />
   
   Where:
   - **xi​,yi**: robot position in global frame.
   - **ψi**: robot orientation (heading).

   Time derivative:
   
   <img width="142" height="33" alt="image" src="https://github.com/user-attachments/assets/d9c00e07-4bee-47e4-a5c4-c5d3ae2c999c" />

---

## Control Inputs (Wheel Speeds)**
   The control inputs are wheel angular velocities:

   <img width="136" height="38" alt="image" src="https://github.com/user-attachments/assets/4c684022-912b-4844-9929-f2530e472274" />
    
   Where:
   - **θ˙iR**: right wheel angular velocity
   - **θ˙iL**: left wheel angular velocity.

   Distance between wheels:
   - **d**: wheelbase

---

## Forward Kinematics (Wheel → Robot Velocity)**
   Robot linear and angular velocities are expressed as:

   <img width="208" height="91" alt="image" src="https://github.com/user-attachments/assets/61368332-63fa-4f98-855a-dbff250bf7a1" />

   Where:
   - **vi​**: robot linear velocity
   - **wi**: robot angular velocity

---

## Inverse Kinematics (Robot Velocity → Wheel Speeds)**
   To compute the wheel angular velocities from robot velocities:

   <img width="219" height="102" alt="image" src="https://github.com/user-attachments/assets/44513f45-72d3-40d7-8abf-2617a0f171c4" />

---

## Velocity Mapping to Global Frame**
   Mapping from robot velocity in body frame to global coordinates:

   <img width="212" height="107" alt="image" src="https://github.com/user-attachments/assets/4654243d-796c-4890-b782-1478708357d6" />

   the global motion is:
   - **x˙i​**: vi ​cos ψi
   - **y˙i**: vi ​sin ψi​
   - **ψ˙i**: wi​

---
