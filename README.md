# Quadrotor Landing

**Main repository:** [edrone-simulator](https://github.com/Project-SwaG/edrone-autonomous-ros)

## Task Breakdown

### Alessandro:
- Spawn a vehicle (any type)
- Create a trajectory for the vehicle
- Land the drone on a stationary vehicle
- **Position Controller:** Use the equations from the slides

### Jacopo:
- **Attitude Controller**
- **Firmware**

## Notes

- **PWM values:** `0 - 1024`
- **Maximum motor force:** `7 Newton`  
  (_Calculated as `1024 / 146.28`, defined in `src/gazebo_edrone_propulsion.ccp` at line 270_)
- **Maximum thrust:** `28 Newton`
- **Hovering thrust:** `9.8 Newton`  
  (_Calculated as `F = m * g = 9.8`, with `m = 1kg`_)

---

## How to Execute

1. Ensure you are using **ROS 1 (Noetic)**.
2. Navigate to the workspace directory:
   ```bash
   cd ~/catkin_ws
   ```
3. Clone the repository into the `/src` folder:
   ```bash
   git clone <repo-url> src/
   ```
4. Build the workspace:
   ```bash
   catkin_make
   ```
5. Source the setup file:
   ```bash
   source devel/setup.bash
   ```
6. Ensure scripts are executable:
   ```bash
   chmod +x <script-python>.py
   ```
7. Ensure they have `#!/usr/bin/env python3` at the top.
8. Launch the main node:
   ```bash
   roslaunch vitarana_drone task_1.launch
   ```

---

If you need any modifications or additional details, feel free to ask! 🚀