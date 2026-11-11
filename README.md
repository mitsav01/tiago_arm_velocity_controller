# TIAGo Arm Velocity Controller

A custom ROS 2 controller for the PAL Robotics TIAGo Dual robot. It accepts joint velocity references and converts them into joint-position setpoints.

The controller is designed for the PAL Robotics Alum environment and ROS 2 Humble.

## 1. Choose your installation method

There are two ways to install or update this controller. Choose the method that matches your situation.

### Option A: Quick Setup — Existing installation

**Who should use this?**

Use Quick Setup if:
- The controller package is already installed on your TIAGo robot.
- The plugin XML and ament resource-index registration are already available.
- You only want to update the compiled shared library (`.so`).
- You do not want to compile the package or set up Docker.

**What you need:** The compatible precompiled `.so` file and SSH access to the robot.

This method only replaces the shared library. It does not perform a fresh package installation.

### Option B: Manual Full Installation — New installation or development

**Who should use this?**

Use Manual Full Installation if:
- You are installing the controller for the first time.
- The controller package or its plugin metadata is not installed.
- You want to build the controller from source.
- You are modifying the controller and need to recompile it.
- You need to deploy the complete package to a new workspace.

**What you need:** The source repository, PAL Alum development Docker image, and access to the TIAGo robot.

This method covers the complete process, from cloning the repository to building, deploying, and verifying the plugin.

## 2. Compatibility

The precompiled library and build instructions target the following environment:

| Component | Version |
|---|---|
| Robot | PAL Robotics TIAGo Dual |
| PAL software | Alum 25.01 |
| ROS 2 | Humble |
| Ubuntu | 22.04 (Jammy) |
| Architecture | x86_64 |

**Important:** The controller must be compiled against the PAL-specific controller and hardware interface libraries. A library compiled against incompatible upstream ROS 2 libraries may fail to load because of ABI incompatibilities.

## 3. Controller overview

The plugin type is:

```text
tiago_velocity_controller_ns/TiagoVelocityController
```

The controller accepts seven joint velocity references, in rad/s, through a `Float64MultiArray` message.

It uses the commanded velocities to calculate position setpoints. It claims joint position command interfaces and reads joint position and velocity state interfaces.

### Controller names

| Arm | Controller |
|---|---|
| Left | `arm_left_velocity_controller` |
| Right | `arm_right_velocity_controller` |

### ROS 2 topics

| Arm | Command topic | State topic |
|---|---|---|
| Left | `/arm_left_velocity_controller/command` | `/arm_left_velocity_controller/controller_state` |
| Right | `/arm_right_velocity_controller/command` | `/arm_right_velocity_controller/controller_state` |

Message type:

```text
std_msgs/msg/Float64MultiArray
```

The command array must contain seven values corresponding to joints 1–7.

---

# Part A — Quick Setup

## 4. Requirements

Before starting, make sure:

- The controller package is already installed on the robot.
- The plugin XML and ament resource-index registration are present.
- You have the compatible `libtiago_arm_velocity_controller.so` file.
- You can connect to the robot through SSH.

The precompiled library must match the robot's PAL Alum, ROS 2, Ubuntu, architecture, and ABI environment.

## 5. Transfer the shared library

Download the compatible `.so` file from the repository's GitHub Releases page.

Alternatively, obtain the library from someone who has compiled it using the PAL Alum development environment.

From your development PC, transfer it to the robot:

```bash
scp libtiago_arm_velocity_controller.so \
  pal@10.0.0.37:/home/pal/
```

Connect to the robot:

```bash
ssh pal@10.0.0.37
```

## 6. Replace the existing shared library

Back up the existing library if present:

```bash
cp ~/deployed_ws/install/lib/libtiago_arm_velocity_controller.so \
   ~/deployed_ws/install/lib/libtiago_arm_velocity_controller.so.bak
```

Replace it with the new library:

```bash
cp ~/libtiago_arm_velocity_controller.so \
   ~/deployed_ws/install/lib/libtiago_arm_velocity_controller.so
```

The destination is:

```text
~/deployed_ws/install/lib/libtiago_arm_velocity_controller.so
```

If the library is not present in the merged installation directory, check the package-specific installation directory:

```text
~/deployed_ws/install/tiago_arm_velocity_controller/lib/
```

In that case, update the library in the location used by the running environment's library search path.

## 7. Source the environment and verify the plugin

Source the PAL and workspace environments:

```bash
source /opt/pal/alum/setup.bash
source ~/deployed_ws/install/setup.bash
```

Verify plugin discovery:

```bash
ros2 control list_controller_types | grep -i tiago
```

Expected plugin:

```text
tiago_velocity_controller_ns/TiagoVelocityController
```

If the controller manager is already running, restart the relevant process before expecting it to load the updated shared library. A running process may continue using the library already loaded in memory.

**Quick Setup is complete.** You can now proceed to Section 12 to load, activate, and test the controller.

---

# Part B — Manual Full Installation

## 8. Development environment

The controller should be compiled inside the PAL Alum development environment to ensure compatibility with the robot.

The recommended approach is to use the Docker development image.

### Required Docker image

```text
development-tiago-dual-140:alum-25.01
```

This image is based on the PAL Robotics development environment:

```text
palrobotics/deploy-alum-25.01-development:latest
```

The `development-tiago-dual-140:alum-25.01` image is a local image tag used by this workflow. Do not assume it is directly available from Docker Hub. Obtain the image from the development environment or the person maintaining the PAL development setup.

## 9. Clone the repository

On the development PC:

```bash
mkdir -p ~/deployed_ws/src
cd ~/deployed_ws/src
```

Clone the repository:

```bash
git clone <REPOSITORY_URL> tiago_arm_velocity_controller
```

If the project is maintained on a dedicated branch, check it out:

```bash
cd tiago_arm_velocity_controller
git checkout tiago-alum
```

The resulting workspace structure should look like:

```text
deployed_ws/
└── src/
    └── tiago_arm_velocity_controller/
        ├── CMakeLists.txt
        ├── package.xml
        ├── include/
        ├── src/
        └── config/
```

## 10. Start the Docker container

Run the following command from the development PC:

```bash
docker run --rm -it \
  --network host \
  --ipc host \
  -v "$HOME/deployed_ws:/workspace" \
  -w /workspace \
  development-tiago-dual-140:alum-25.01 \
  bash
```

This mounts the local workspace inside the container at `/workspace`.

### Source ROS 2 and PAL Alum

Inside the Docker container:

```bash
source /opt/ros/humble/setup.bash
source /opt/pal/alum/setup.bash
```

Verify the ROS distribution:

```bash
echo $ROS_DISTRO
```

Expected output:

```text
humble
```

## 11. Compile and deploy the controller

### 11.1 Compile the package

From `/workspace`, build the package:

```bash
colcon build \
  --packages-select tiago_arm_velocity_controller \
  --event-handlers console_direct+ \
  --cmake-args \
    -Dcontroller_interface_DIR=/opt/pal/alum/share/controller_interface/cmake \
    -Dhardware_interface_DIR=/opt/pal/alum/share/hardware_interface/cmake \
    -Drealtime_tools_DIR=/opt/pal/alum/share/realtime_tools/cmake
```

The package-specific installation will be generated under:

```text
install/tiago_arm_velocity_controller/
```

The compiled shared library should be located at:

```text
install/tiago_arm_velocity_controller/lib/libtiago_arm_velocity_controller.so
```

### 11.2 Verify the compiled library

Check the shared library for unresolved symbols:

```bash
ldd -r install/tiago_arm_velocity_controller/lib/libtiago_arm_velocity_controller.so
```

Confirm that dependencies resolve against the PAL Alum environment and that there are no unresolved symbols.

Do not deploy a library that has unresolved dependencies or was built against an incompatible controller interface ABI.

### 11.3 Deploy the complete package to TIAGo

For a fresh installation or when the package metadata also needs updating, transfer the complete package-specific installation:

```bash
rsync -av \
  ~/deployed_ws/install/tiago_arm_velocity_controller/ \
  pal@10.0.0.37:/home/pal/deployed_ws/install/tiago_arm_velocity_controller/
```

The robot's merged library directory may also need the correct shared library, depending on the workspace and library search path:

```bash
cp \
  ~/deployed_ws/install/tiago_arm_velocity_controller/lib/libtiago_arm_velocity_controller.so \
  ~/deployed_ws/install/lib/libtiago_arm_velocity_controller.so
```

On the robot, source the environment:

```bash
source /opt/pal/alum/setup.bash
source ~/deployed_ws/install/setup.bash
```

### 11.4 Verify plugin discovery

Run:

```bash
ros2 control list_controller_types | grep -i tiago
```

Expected output:

```text
tiago_velocity_controller_ns/TiagoVelocityController
```

---

# Part C — Controller Operation

## 12. Load and activate the controller

The controller can be loaded using the ROS 2 controller manager.

### Load the left arm controller

```bash
ros2 control load_controller \
  arm_left_velocity_controller \
  -c /controller_manager
```

### Load the right arm controller

```bash
ros2 control load_controller \
  arm_right_velocity_controller \
  -c /controller_manager
```

Check controller status:

```bash
ros2 control list_controllers -c /controller_manager
```

### Activate the left arm controller

```bash
ros2 control switch_controllers \
  --activate arm_left_velocity_controller \
  -c /controller_manager
```

### Activate the right arm controller

```bash
ros2 control switch_controllers \
  --activate arm_right_velocity_controller \
  -c /controller_manager
```

**Important:** Ensure that conflicting position or gravity compensation controllers are not active before activating the velocity controller. Follow the robot's safety procedures when switching controllers.

## 13. Send velocity commands

The controller accepts seven joint velocity references in rad/s.

Example command for the left arm:

```bash
ros2 topic pub --once \
  /arm_left_velocity_controller/command \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

This sends a velocity reference of `0.02 rad/s` to the first joint and zero to the remaining six joints.

Before sending commands, ensure that:

- The correct controller is active.
- No conflicting controller is active.
- The robot's workspace is clear.
- The arm is away from joint limits and obstacles.
- The emergency stop is accessible.
- Joint motion is monitored during testing.

Start with small velocity references and test one arm at a time.

## 14. Troubleshooting

### Plugin not found

Check whether the plugin is registered:

```bash
ros2 control list_controller_types | grep -i tiago
```

Verify that the package's plugin XML and ament resource-index registration are installed and that the workspace has been sourced.

### Shared library loading error

Inspect the library dependencies:

```bash
ldd -r ~/deployed_ws/install/lib/libtiago_arm_velocity_controller.so
```

Look for unresolved symbols or dependencies that are not found.

A common cause is compiling against upstream ROS 2 Humble libraries instead of the PAL Alum-specific libraries.

### Controller fails to load

Check the controller manager output and verify that:

- The plugin type is correct.
- The controller configuration is installed.
- The required joint interfaces are available.
- The shared library is located in the active library search path.
- The library is compatible with the installed PAL Alum environment.

### Updated library does not take effect

A running process may still have the previous shared library loaded. Restart the relevant controller manager or robot software process before testing the updated library.

## 15. Binary release guidelines

When publishing a precompiled shared library, clearly identify its target environment.

Recommended release filename:

```text
libtiago_arm_velocity_controller-alum-25.01-humble-ubuntu22.04-x86_64.so
```

A release should include:

- The compiled shared library.
- The supported PAL Alum and ROS 2 versions.
- The target Ubuntu version and CPU architecture.
- A checksum for verifying the downloaded file.

Users with an existing installation can then update the controller by replacing the `.so` file, without rebuilding the package or copying the complete installation tree.

**Compatibility warning:** A precompiled library is only suitable for an environment with compatible ROS 2, PAL Alum, compiler ABI, and dependent libraries. Always validate it on the target robot before using it for physical motion.