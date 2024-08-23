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


## 2. [Cartpole](https://leggedrobotics.github.io/ocs2/robotic_examples.html#cartpole)

* build
```bash
cd ../..
colcon build --packages-up-to ocs2_cartpole_ros --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
```
* run
```bash
source ../../install/setup.bash
ros2 launch ocs2_cartpole_ros cartpole.launch.py
```


## 3 [Ballbot](https://leggedrobotics.github.io/ocs2/robotic_examples.html#ballbot)

* build
```bash
cd ~/ocs2_ws
colcon build --packages-up-to ocs2_ballbot_ros --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
```
* run
```bash
source ~/ocs2_ws/install/setup.bash
ros2 launch ocs2_ballbot_ros ballbot_ddp.launch.py
```
