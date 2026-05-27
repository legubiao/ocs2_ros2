# OCS2_ROS2 Toolbox

## 1. Summary

OCS2_ROS2 is developed based on [OCS2](https://github.com/leggedrobotics/ocs2), it was refactored to be compatible with ROS2 and modern cmake.

### What's New (2026.05)

**ROS 2 Lyrical Support**
- Add support for ROS 2 Lyrical on Ubuntu 26.04.
- Modernize launch files for newer `robot_state_publisher`: URDF files are now parsed and passed through the `robot_description` parameter instead of the removed positional URDF path argument.
- Replace deprecated `tf2` / `tf2_ros` `.h` includes with `.hpp` headers for Jazzy and Lyrical compatibility.
- Remove dependency on the removed `urdf::exportURDF` API by keeping the original URDF XML inside the Pinocchio interface.
- Make RViz plugins select the matching Qt major version automatically (`Qt5` on Jazzy, `Qt6` on Lyrical).
- Avoid OctoMap ABI conflicts by letting the collision backend own its native OctoMap linkage.
- Default launch terminal prefix is now `xterm -e`, which works better in WSL and container environments such as distrobox. Override it with `OCS2_TERMINAL_PREFIX` if needed.

### What's New (2026.01)

**Environment Collision for Mobile Manipulator**
- Add basic geometry environment collision support.
- Please check Franka demo for more details.

### Tested Platform

* Intel Nuc X15 (i7-11800H):
    * Ubuntu 22.04 ROS2 Humble  (WSL2 included)
    * Ubuntu 24.04 ROS2 Jazzy   (WSL2 included)
    * Ubuntu 26.04 ROS2 Lyrical (distrobox)
* Lenovo P16v (i7-13800H):
    * Ubuntu 24.04 ROS2 Jazzy
* Jetson Orin Nano
    * Ubuntu 22.04 ROS2 Humble (JetPack 6.1)

## 2. Installation

### 2.1 Prerequisites

The OCS2 library is written in C++17. It is tested under Ubuntu with library versions as provided in the package
sources.

Tested system and ROS2 version:

* Ubuntu 26.04 ROS2 Lyrical
* Ubuntu 24.04 ROS2 Jazzy
* Ubuntu 22.04 ROS2 Humble

> **Note:** Some demos open auxiliary nodes in a new terminal. The default terminal prefix is `xterm -e`
> for better compatibility with WSL and distrobox. Install it if necessary:
> ```bash
> sudo apt install xterm
> ```
> You can override the terminal prefix, for example:
> ```bash
> export OCS2_TERMINAL_PREFIX="gnome-terminal --"
> ```

### 2.2 Dependencies

* C++ compiler with C++17 support
* Eigen (v3.4)
* Boost C++ (v1.74)

> **Note:** Latest version used pinocchio from ros source to simplified install steps. If you install pinocchio from robot-pkgs, you can uninstall it by
> ```bash
> sudo apt remove robotpkg-*
> ```

### 2.3 Install from deb (try basic examples without building)

Prebuilt `.deb` packages bundle the OCS2 core stack and **all basic examples** listed in [Section 3](#3-basic-examples). This is the fastest way to run the demos if you do not need to modify the source code.

| ROS 2 distro | Ubuntu | Debian package | Install prefix |
|--------------|--------|----------------|----------------|
| Jazzy | 24.04 (Noble) | `ros-jazzy-ocs2` | `/opt/ros/jazzy` |
| Lyrical | 26.04 (Resolute) | `ros-lyrical-ocs2` | `/opt/ros/lyrical` |

**Prerequisites**

1. Install the matching ROS 2 base from [docs.ros.org](https://docs.ros.org/) (e.g. `ros-jazzy-ros-base` or `ros-lyrical-ros-base`).
2. Install `xterm` (used by several example launch files to open MPC / dummy nodes in separate terminals):
   ```bash
   sudo apt install xterm
   ```

**Download and install**

Pick the asset that matches your distro and CPU architecture (`amd64` or `arm64`) from the [GitHub Releases](https://github.com/legubiao/ocs2_ros2/releases) page.

Example for **ROS 2 Jazzy** on amd64 (replace `VERSION` with the release tag, e.g. `1.0.0`):

```bash
wget https://github.com/legubiao/ocs2_ros2/releases/download/vVERSION/ros-jazzy-ocs2_VERSION_amd64.deb
sudo dpkg -i ros-jazzy-ocs2_VERSION_amd64.deb
sudo apt-get install -f   # if apt reports missing dependencies
```

Example for **ROS 2 Lyrical** on amd64:

```bash
wget https://github.com/legubiao/ocs2_ros2/releases/download/vVERSION/ros-lyrical-ocs2_VERSION_amd64.deb
sudo dpkg -i ros-lyrical-ocs2_VERSION_amd64.deb
sudo apt-get install -f
```

**Use the installed packages**

```bash
# Jazzy:
source /opt/ros/jazzy/setup.bash

# Lyrical:
# source /opt/ros/lyrical/setup.bash
```

Then follow the **run** steps in [Section 3](#3-basic-examples) (skip the **build** steps). For example:

```bash
ros2 launch ocs2_double_integrator_ros double_integrator.launch.py
```

To confirm the install:

```bash
ros2 pkg list | grep ocs2_
dpkg -L ros-jazzy-ocs2    # or ros-lyrical-ocs2
```

> **Note:** The deb installs into the same prefix as your system ROS packages. You only need to `source /opt/ros/<distro>/setup.bash` once per shell. Advanced examples under `advance examples/` are **not** included in the deb; build those from source if needed ([Section 2.4](#24-clone-repositories)).

### 2.4 Clone Repositories

* Create a new workspace or clone the project to your workspace

```bash
cd ~
mkdir -p ros2_ws/src
```

* Clone the repository

```bash
cd ~/ros2_ws/src
git clone https://github.com/legubiao/ocs2_ros2
cd ocs2_ros2
git submodule update --init --recursive
```

* rosdep

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

## 3. Basic Examples

This section contains basic examples for the OCS2 library.

If you installed the prebuilt deb ([Section 2.3](#23-install-from-deb-try-basic-examples-without-building)), **skip the build steps** in each example: `source /opt/ros/<distro>/setup.bash` and run the `ros2 launch` command directly. To build from source instead, use [Section 2.4](#24-clone-repositories) and the build commands below.

### 3.1 [Double Integrator](https://leggedrobotics.github.io/ocs2/robotic_examples.html#double-integrator)

<details>
<summary>🎯 Click to expand Double Integrator example</summary>

* build
```bash
cd ~/ros2_ws
colcon build --packages-up-to ocs2_double_integrator_ros --symlink-install
```
* run
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ocs2_double_integrator_ros double_integrator.launch.py
```

https://github.com/user-attachments/assets/581d03ff-43e4-49c9-8f47-a0ce491b585c

</details>

### 3.2 [Cartpole](https://leggedrobotics.github.io/ocs2/robotic_examples.html#cartpole)

<details>
<summary>🛒 Click to expand Cartpole example</summary>

* build
```bash
cd ~/ros2_ws
colcon build --packages-up-to ocs2_cartpole_ros --symlink-install
```
* run
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ocs2_cartpole_ros cartpole.launch.py
```

https://github.com/user-attachments/assets/7fe0fe18-3ad5-47dd-9fe2-be90413c2f2f

</details>

### 3.3 [Ballbot](https://leggedrobotics.github.io/ocs2/robotic_examples.html#ballbot)

<details>
<summary>🏀 Click to expand Ballbot example</summary>

* build
```bash
cd ~/ros2_ws
colcon build --packages-up-to ocs2_ballbot_ros --symlink-install
```
* run
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ocs2_ballbot_ros ballbot_ddp.launch.py
```

https://github.com/user-attachments/assets/c87966b8-525f-4592-a54f-cfaed458a6f2

</details>

### 3.4 [Quadrotor](https://leggedrobotics.github.io/ocs2/robotic_examples.html#quadrotor)

<details>
<summary>🚁 Click to expand Quadrotor example</summary>

* build
```bash
cd ~/ros2_ws
colcon build --packages-up-to ocs2_quadrotor_ros --symlink-install
```
* run
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ocs2_quadrotor_ros quadrotor.launch.py
```

https://github.com/user-attachments/assets/aed3173f-a6e6-4499-ae8c-d101bedc5222

</details>

### 3.5 [Mobile Manipulator](https://leggedrobotics.github.io/ocs2/robotic_examples.html#mobile-manipulator)

<details>
<summary>🦾 Click to expand Mobile Manipulator example</summary>

* build
```bash
cd ~/ros2_ws
colcon build --packages-up-to ocs2_mobile_manipulator_ros --symlink-install
```
* run Mabi-Mobile
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ocs2_mobile_manipulator_ros manipulator_mabi_mobile.launch.py
```

https://github.com/user-attachments/assets/c71f6123-fa3a-4b72-a60f-5509b8c25413

* run Kinova Jaco2
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ocs2_mobile_manipulator_ros manipulator_kinova_j2n6.launch.py
```
* run Franka Panda
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ocs2_mobile_manipulator_ros franka.launch.py
```

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ocs2_mobile_manipulator_ros franka_sqp.launch.py
```

https://github.com/user-attachments/assets/bab14b46-486e-46dc-a268-bd63616d1010

* run Willow Garage PR2
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ocs2_mobile_manipulator_ros pr2.launch.py
```

https://github.com/user-attachments/assets/100aae62-9e80-487b-89cf-ea6a97ef2505

* run Clearpath Ridgeback with UR-5
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ocs2_mobile_manipulator_ros manipulator_ridgeback_ur5.launch.py 
```

</details>

### 3.6 [Legged Robot](https://leggedrobotics.github.io/ocs2/robotic_examples.html#legged-robot)

<details>
<summary>🐕 Click to expand Legged Robot example</summary>

* build
```bash
cd ~/ros2_ws
colcon build --packages-up-to ocs2_legged_robot_ros --symlink-install
```
* run
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ocs2_legged_robot_ros legged_robot_ddp.launch.py
```

https://github.com/user-attachments/assets/d29551b7-2ac7-428d-9605-f782193bcaf2

</details>

## 4. Advanced Examples

[![](http://i1.hdslb.com/bfs/archive/a53bab50141165eb452aa0763a9a5b9a51a7ca67.jpg)](https://www.bilibili.com/video/BV1gSHLe3EEv/)

### 4.1 [Perceptive Locomotion](advance%20examples/ocs2_perceptive_anymal/)

![perceptive_side](.images/perception_side.png)

![perceptive_hurdles](.images/perception_hurdles.png)

### 4.2 [RaiSim Simulation](advance%20examples/ocs2_raisim/)

![raisim](.images/raisim.png)

![raisim_rviz](.images/raisim_rviz.png)

### 4.3 [MPC-Net](advance%20examples/ocs2_mpcnet/)

## 5. Related Projects

* [quadruped ros2 control](https://github.com/legubiao/quadruped_ros2_control)： Quadruped controller based on OCS2 ROS2
* [arms ro2 control](https://github.com/fiveages-sim/arms_ros2_control): Mobile manipulator controller based on OCS2 ROS2
* [robot_descriptions](https://github.com/fiveages-sim/robot_descriptions): More robot configs for OCS2 ROS2