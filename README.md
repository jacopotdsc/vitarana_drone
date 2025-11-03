# 🚁 Autonomous Quadrotor Landing on a Moving Vehicle

This project presents a complete control and simulation framework for the autonomous landing of a quadrotor on a moving vehicle, including a collision avoidance module.

## 📌 Abstract

A complete control and simulation framework was developed for the autonomous landing of a quadrotor on a moving car, with the addition of a collision avoidance module. The project included modeling, controller design, and simulation validation in the Gazebo environment, using ground truth data. The implemented architecture allows the drone to synchronize with the vehicle's trajectory and perform a stable and precise landing. The multi-layer control system integrates position and attitude controllers, a vehicle controller, and a collision avoidance module to ensure a safe and stable landing.

## 🚀 Usage

* **Branch: main**
    To launch the standard landing simulation (without obstacles):
    ```bash
    roslaunch car_and_drone.launch
    ```

* **Branch: collision_avoidance**: to launch the simulation with a static obstacle:
        ```bash
        roslaunch obstacle.launch
        ```
*  **Branch: obstacle_traj**: to launch the simulation with an obstacle and a predefined trajectory:
        ```bash
        roslaunch obstacle_traj.launch
        ```

## 📐 Problem Formulation

The project's goal is to bring the drone to the car's position. Given the drone's position $p_{R}(t)$ and the desired position (the car) $p_{d}(t)$, the objective is:

$$lim_{t\rightarrow\infty}||p_{R}(t)-p_{d}(t)||=0$$

This problem is formulated as a regulation problem. We define the position error $e_{p}(t)$ and the velocity error $e_{v}(t)$ as:

$$e_{p}(t)=p_{R}(t)-p_{d}(t)$$
$$e_{v}(t)=\dot{p}_{R}(t)-\dot{p}_{d}(t)$$

The goal is to drive both errors to zero over time:

$$lim_{t\rightarrow\infty}||e_{p}(t)||=0$$
$$lim_{t\rightarrow\infty}||e_{v}(t)||=0$$

## ⚙️ Controllers

The control architecture is based on several interconnected modules:

### Position Controller

This controller computes the Cartesian thrust needed to reach the desired position. It uses a PD approach:

$$T_{x}=K_{p}(x_{des}-x)+K_{d}(\dot{x}_{des}-\dot{x})$$
$$T_{y}=K_{p}(y_{des}-y)+K_{d}(\dot{y}_{des}-\dot{y})$$
$$T_{z}=K_{p}(z_{des}-y)+K_{d}(\dot{z}_{des}-\dot{y})$$

### Attitude Controller

Responsible for stabilizing the quadrotor's orientation, it computes the attitude torques to reach the desired orientation. This also uses PD control:

$$\tau_{\phi}=K_{p}(\phi_{des}-\phi)+K_{d}(\dot{\phi}_{des}-\dot{\phi})$$
$$\tau_{\theta}=K_{p}(\theta_{des}-\theta)+K_{d}(\dot{\theta}_{des}-\dot{\theta})$$
$$\tau_{\psi}=K_{p}(\psi_{des}-\psi)+K_{d}(\dot{\psi}_{des}-\dot{\psi})$$

### Car Controller

The car's control is based on a Virtual Target Point (VTP) to follow the predefined trajectory.

### Collision Avoidance

To ensure safety in complex environments, a collision avoidance module was implemented. This module calculates a repulsive force and a tangential force to avoid detected obstacles.

* **Repulsive Force:** $||f_{rep}||=K_{ox}w(d)\frac{1}{max\{d,d_{safe}\}}$
* **Tangential Force:** $||f_{tan}||=K_{oy}w(d)\frac{1}{min\{1,|c|\}}$

## Future Works

Future work will focus on integrating state estimation modules to replace ground truth data, thereby enabling real-world testing.

## 📚 References

[1] Jawhar Ghommam and Maarouf Saad. Autonomous landing of a quadrotor on a moving platform. *IEEE Transactions on Aerospace and Electronic Systems*, 2020.

[2] Lecture Notes from the course Modeling and Control of Multi-Rotor UAVs. Lecture notes on modeling and control of multi-rotor uavs, 2024/2025. University course, personal notes and slides.

[3] Lecture Notes from the course Aerial Robotics for Physical Interaction and Manipulation. Lecture notes on aerial robotics for physical interaction and manipulation, 2024/2025. University course, personal notes and slides.

[4] Lecture Notes from the course Autonomous and Mobile Robotics. Lecture notes on autonomous and mobile robotics, 2024/2025. University course, personal notes and slides.

## 👥 Authors

Jacopo Tedeschi([jacopotdsc](https://github.com/jacopotdsc)), Alessandro Angelo Anzellini([Alexanderis1](https://github.com/Alexanderis1))