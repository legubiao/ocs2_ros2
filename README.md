# OCS2_ROS2 Toolbox
<script src="//unpkg.com/docsify-bilibili/bilibili.min.js"></script>

## 1. Summary
OCS2_ROS2 is developed based on [OCS2](https://github.com/leggedrobotics/ocs2), it was refactored to be compatible with ROS2 and modern cmake. Below is the current todolist of the project:

- [x] modern cmake
- [x] basic 6 examples in official documents
- [x] WSL2 support
- [x] Fix Quadrotor example's unexpected behavior
- [x] Fix Mobile Manipolator's interactive marker
- [x] perceptive locomotion demo
- [x] tinyxml2 problem in Ubuntu 24.04 ROS2 Jazzy
- [x] raisim demo
- [ ] mpc_net demo

The IDE I used is CLion, you can follow the [guide](https://www.jetbrains.com/help/clion/ros2-tutorial.html) to set up the IDE.

## 2. Installation
### 2.1 Prerequisites
The OCS2 library is written in C++17. It is tested under Ubuntu with library versions as provided in the package sources.

Tested system and ROS2 version:
* Ubuntu 24.04 ROS2 Jazzy
* Ubuntu 22.04 ROS2 Humble

### 2.2 Dependencies
* C++ compiler with C++17 support
* Eigen (v3.4)
* Boost C++ (v1.74)

### 2.3 Clone Repositories
* Create a new workspace or clone the project to your workspace
```bash
cd ~
mkdir -p ocs2_ws/src
```
* Clone the repository

```bash
cd ~/ocs2_ws/src
git clone https://github.com/legubiao/ocs2_ros2
git submodule update --init --recursive
```
* rosdep
```bash
cd ~/ocs2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 2.4 Basic Examples

- Bilibili video 
- bvid=BV12vv9eGEns


Check [here](basic%20examples/) for more detailed instructions.

## 3. Advanced Examples

### 3.1 Perceptive Locomotion

[OCS2_Perceptive Anymal](advance%20examples/ocs2_perceptive_anymal/)

### 3.2 RaiSim Simulation

[OCS2_RaiSim](advance%20examples/ocs2_raisim/)