# OCS2_ROS2 Toolbox

## 1. Summary

OCS2_ROS2 is developed based on [OCS2](https://github.com/leggedrobotics/ocs2), it was refactored to be compatible with ROS2 and modern cmake.

### What's New (2025.08)

**Pinocchio 3 Dependency Optimization**
- Upgraded to Pinocchio 3 version for better performance and stability
- Support for installing Pinocchio from ROS sources, avoiding complex third-party package management

**Dual-Arm Mobile Manipulator Support**
- Added Dual-Arm Mobile Manipulator functionality
- Enhanced interactive markers for better user operation experience

The IDE I used is CLion, you can follow the [guide](https://www.jetbrains.com/help/clion/ros2-tutorial.html) to set up
the IDE.

### Tested Platform

* Intel Nuc X15 (i7-11800H):
    * Ubuntu 22.04 ROS2 Humble  (WSL2 included)
    * Ubuntu 24.04 ROS2 Jazzy   (WSL2 included)
* Lenovo P16v (i7-13800H):
    * Ubuntu 24.04 ROS2 Jazzy
* Jetson Orin Nano
    * Ubuntu 22.04 ROS2 Humble (JetPack 6.1)

## 2. Installation

### 2.1 Prerequisites

The OCS2 library is written in C++17. It is tested under Ubuntu with library versions as provided in the package
sources.

Tested system and ROS2 version:

* Ubuntu 24.04 ROS2 Jazzy
* Ubuntu 22.04 ROS2 Humble

### 2.2 Dependencies

* C++ compiler with C++17 support
* Eigen (v3.4)
* Boost C++ (v1.74)

> **Note:** Latest version used pinocchio from ros source to simplified install steps. If you install pinocchio from robot-pkgs, you can uninstall it by
> ```bash
> sudo apt remove robotpkg-*
> ```

### 2.3 Clone Repositories

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

* run Willow Garage PR2
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ocs2_mobile_manipulator_ros pr2.launch.py
```

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