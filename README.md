# OCS2_ROS2 Toolbox

## Summary
OCS2_ROS2 is developed based on [OCS2](https://github.com/leggedrobotics/ocs2), and features that are not supported at the moment:

* ocs2_mpcnet
* ocs2_doc

## Installation
### Prerequisites
The OCS2 library is written in C++17. It is tested under Ubuntu with library versions as provided in the package sources.

Tested system and ROS2 version:
* Ubuntu 24.04 ROS2 Jazzy
* Ubuntu 22.04 ROS2 Humble

### Dependencies
* C++ compiler with C++17 support
* Eigen (v3.4)
* Boost C++ (v1.74)

### Installation Setps
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
* [grid_map](https://github.com/ANYbotics/grid_map) (jazzy only 2024.8.1)
```bash
git clone https://github.com/ANYbotics/grid_map.git
cd grid_map
git checkout jazzy
cd ../..
colcon build --packages-up-to grid_map --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
```
#### Build Examples
**⚠️ Warning:**

If build without "-DCMAKE_BUILD_TYPE=RelWithDebInfo", the mpc will have poor performance.
##### [Double Integrator](https://leggedrobotics.github.io/ocs2/robotic_examples.html#double-integrator)

* build
```bash
colcon build --packages-up-to ocs2_double_integrator_ros --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
```
* run
```bash
cd ~/ocs2_ws
source install/setup.bash
ros2 launch ocs2_double_integrator_ros double_integrator.launch.py
```

##### [Cartpole](https://leggedrobotics.github.io/ocs2/robotic_examples.html#cartpole)

* build
```bash
colcon build --packages-up-to ocs2_cartpole_ros --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
```
* run
```bash
cd ~/ocs2_ws
source install/setup.bash
ros2 launch ocs2_cartpole_ros cartpole.launch.py
```



##### [Legged Robot](https://leggedrobotics.github.io/ocs2/robotic_examples.html#legged-robot)
```bash
colcon build --packages-up-to ocs2_legged_robot_ros ocs2_self_collision_visualization --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
```
