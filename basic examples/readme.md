# OCS2 Basic Examples

This folder contains basic examples for the OCS2 library.

## 1. [Double Integrator](https://leggedrobotics.github.io/ocs2/robotic_examples.html#double-integrator)

* build
```bash
cd ~/ocs2_ws
colcon build --packages-up-to ocs2_double_integrator_ros --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
```
* run
```bash
source ~/ocs2_ws/install/setup.bash
ros2 launch ocs2_double_integrator_ros double_integrator.launch.py
```