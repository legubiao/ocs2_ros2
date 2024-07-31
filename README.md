# OCS2_ROS2 Toolbox

## Summary
OCS2_ROS2 is developed based on [OCS2](https://github.com/leggedrobotics/ocs2), and features that are not supported at the moment:

* ocs2_mpcnet
* ocs2_doc

## Installation
### Prerequisites
The OCS2 library is written in C++17. It is tested under Ubuntu 22.04 with library versions as provided in the package sources.

### Dependencies
* C++ compiler with C++17 support
* ros2 iron
* Eigen (v3.4)
* Boost C++ (v1.74)
* For rigid multi-body dynamics library and self collision support clone Pinocchio and HPP-FCL into your workspace
```
# install pinocchio
git clone --recurse-submodules https://github.com/zhengxiang94/pinocchio.git
# install hpp-fcl
git clone --recurse-submodules https://github.com/zhengxiang94/hpp-fcl.git
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
```
* rosdep
```bash
rosdep install --from-paths src --ignore-src -r -y
```
* others
```
sudo apt-get install ros-iron-grid-map-cv ros-iron-grid-map-msgs ros-iron-grid-map-ros ros-iron-grid-map-sdf libmpfr-dev libpcap-dev
```

