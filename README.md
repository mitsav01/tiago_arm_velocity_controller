# tiago_arm_velocity_controller

A custom `ros2_control` velocity-resolved position controller plugin for the PAL Robotics TIAGo Dual robot arm (7-DoF) in ROS 2 Jazzy.

---

## Overview

Industrial and collaborative manipulators like the PAL Robotics TIAGo often run closed-loop position mode at the hardware level, rejecting direct velocity or torque commands.

This package provides an integrating position controller. It accepts continuous joint velocity commands ($\text{rad/s}$) over a standard ROS 2 topic and integrates them into position setpoints ($\text{rad}$) in the 500 Hz real-time loop:

$$x_{\text{target}}(t + \Delta t) = x_{\text{target}}(t) + v_{\text{cmd}} \cdot \Delta t$$

The integrated position setpoints are written directly to `hardware_interface::HW_IF_POSITION`, bypassing software PID velocity tuning while leveraging the low-level motor drives for drift-free holding and trajectory tracking.

---

## Key Features

* **Pure Integrating Control:** Converts input velocities directly to position targets without software PID loops or gain tuning.
* **Fail-Safe Watchdog:** Automatically halts motion if command messages cease for longer than the timeout period (default: `200 ms`).
* **URDF Joint Limit Protection:** Reads position limits directly from the URDF model and clamps integrated targets before sending them to the hardware interface.
* **Bumpless Transfer:** Latches physical joint angles upon activation to eliminate startup jerks.
* **Real-Time Safe:** Employs lock-free double buffering (`realtime_tools::RealtimeBuffer`) with zero dynamic heap allocations in the critical update loop.
* **Dual-Arm Support:** Joints are specified via parameters, allowing independent or simultaneous control of both left and right arms.

---

## Prerequisites

* **OS:** Ubuntu 24.04 LTS
* **ROS 2:** Jazzy Jalisco

---

## Build & Verification

Build the package in your workspace:

```bash
cd ~/tiago_ws
colcon build --packages-select tiago_arm_velocity_controller
source ~/tiago_ws/install/setup.bash
```

---

## Usage

### 1. Activating the Controller

Switch from the default trajectory controller to the velocity controller:

```bash
ros2 launch tiago_arm_velocity_controller switch_controller.launch.py
```

Verify controller status:

```bash
ros2 control list_controllers
```

---

### 2. Sending Velocity Commands

Publish 7-DoF joint velocities ($\text{rad/s}$) to `/arm_left_velocity_controller/commands`:

#### Continuous Streaming (Teleoperation)

Stream velocity setpoints at 10 Hz (e.g., joint 1 moving at $+0.05\text{ rad/s}$):

```bash
ros2 topic pub -r 10 /arm_left_velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

*(Stopping the stream with `Ctrl+C` will trigger the watchdog and halt the arm within 200 ms.)*


#### Hold Position / Immediate Stop

```bash
ros2 topic pub --once /arm_left_velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

