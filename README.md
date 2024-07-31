# OCS2_ROS2 Toolbox

## Summary
OCS2_ROS2 is developed based on [OCS2](https://github.com/leggedrobotics/ocs2), and features that are not supported at the moment:

* ocs2_mpcnet
* ocs2_doc

## Installation
### Prerequisites
The OCS2 library is written in C++17. It is tested under Ubuntu 24.04 with library versions as provided in the package sources.

### Dependencies
* C++ compiler with C++17 support
* ros2 jazzy
* Eigen (v3.4)
* Boost C++ (v1.74)
* For rigid multi-body dynamics library and self collision support clone Pinocchio into your workspace
```
# install pinocchio
git clone --recurse-submodules https://github.com/zhengxiang94/pinocchio.git
```
* For various robotic assets used in OCS2 unit tests and the robotic examples
```
# Clone ocs2_robotic_assets in ros2_ws/src
git clone https://github.com/zhengxiang94/ocs2_robotic_assets.git
```
* plane_segmentation_ros2
```
# Clone plane_segmentation_ros2 in ros2_ws/src
git clone https://github.com/zhengxiang94/plane_segmentation_ros2.git
```
* [grid_map](https://github.com/ANYbotics/grid_map) (jazzy branch)
```bash
git clone https://github.com/ANYbotics/grid_map.git
cd grid_map
git checkout jazzy
cd ../..
colcon build --packages-up-to grid_map --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
```
* rosdep
```bash
rosdep install --from-paths src --ignore-src -r -y
```

### Build
* Only build essential for legged_robot example.
```bash
# If build without "-DCMAKE_BUILD_TYPE=RelWithDebInfo", the mpc will have poor performance.
colcon build --packages-up-to ocs2_legged_robot_ros ocs2_self_collision_visualization --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
```
