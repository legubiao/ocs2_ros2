# OCS2_ROS2 Toolbox

## 1. Summary
OCS2_ROS2 is developed based on [OCS2](https://github.com/leggedrobotics/ocs2), and features that are not supported at the moment:

* ocs2_mpcnet
* ocs2_doc

Todo List:
- [x] modern cmake
- [x] basic 6 examples in official documents
- [x] WSL2 support
- [x] Fix Quadrotor example's unexpected behavior
- [x] Fix Mobile Manipolator's interactive marker
- [ ] perceptive locomotion demo
- [ ] tinyxml2 problem in Ubuntu 24.04 ROS2 Jazzy

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

#### 2.3.1 Extra steps for Ubuntu 24.04 ROS2 Jazzy
Since the grid_map package is not released yet in ROS2 Jazzy, you need to build it from source. 
* [grid_map](https://github.com/ANYbotics/grid_map)
```bash
git clone https://github.com/ANYbotics/grid_map.git
cd grid_map
git checkout jazzy
cd ../..
colcon build --packages-up-to grid_map --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
```
### 2.4 Build Basic Robotic Examples
**⚠️ Warning:**

* **If build without "-DCMAKE_BUILD_TYPE=RelWithDebInfo", the mpc will have poor performance.**
* **Don't forget to deactivate conda before "colcon build", otherwise gtest would generate some file caused later build all failed and you may need to clean the ros2 workspace to recover.**


#### 2.4.1 [Double Integrator](https://leggedrobotics.github.io/ocs2/robotic_examples.html#double-integrator)

* build
```bash
cd ../..
colcon build --packages-up-to ocs2_double_integrator_ros --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
```
* run
```bash
source ../../install/setup.bash
ros2 launch ocs2_double_integrator_ros double_integrator.launch.py
```

#### 2.4.2 [Cartpole](https://leggedrobotics.github.io/ocs2/robotic_examples.html#cartpole)

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


#### 2.4.3 [Ballbot](https://leggedrobotics.github.io/ocs2/robotic_examples.html#ballbot)

* build
```bash
cd ../..
colcon build --packages-up-to ocs2_ballbot_ros --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
```
* run
```bash
source ../../install/setup.bash
ros2 launch ocs2_ballbot_ros ballbot_ddp.launch.py
```

#### 2.4.4 [Quadrotor](https://leggedrobotics.github.io/ocs2/robotic_examples.html#quadrotor)

* build
```bash
cd ../..
colcon build --packages-up-to ocs2_quadrotor_ros --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
```
* run
```bash
source ../../install/setup.bash
ros2 launch ocs2_quadrotor_ros quadrotor.launch.py
```

#### 2.4.5 [Mobile Manipulator](https://leggedrobotics.github.io/ocs2/robotic_examples.html#mobile-manipulator)

* build
```bash
cd ../..
colcon build --packages-up-to ocs2_mobile_manipulator_ros --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
```
* run Mabi-Mobile
```bash
source ../../install/setup.bash
ros2 launch ocs2_mobile_manipulator_ros manipulator_mabi_mobile.launch.py
```
* run Kinova Jaco2
```bash
source ../../install/setup.bash
ros2 launch ocs2_mobile_manipulator_ros manipulator_kinova_j2n6.launch.py
```
* run Franka Panda
```bash
source ../../install/setup.bash
ros2 launch ocs2_mobile_manipulator_ros manipulator_franka.launch.py
```
* run Willow Garage PR2
```bash
source ../../install/setup.bash
ros2 launch ocs2_mobile_manipulator_ros manipulator_pr2.launch.py
```
* run Clearpath Ridgeback with UR-5
```bash
source ../../install/setup.bash
ros2 launch ocs2_mobile_manipulator_ros manipulator_ridgeback_ur5.launch.py 
```


#### 2.4.6 [Legged Robot](https://leggedrobotics.github.io/ocs2/robotic_examples.html#legged-robot)
* build
```bash
cd ../..
colcon build --packages-up-to ocs2_legged_robot_ros ocs2_self_collision_visualization --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
```
* run
```bash
source ../../install/setup.bash
ros2 launch ocs2_legged_robot_ros legged_robot_ddp.launch.py
```

## 3. Advanced Examples

### 3.1 Perceptive Locomotion

Before tried this example, you need to clone the [Plane Segmentation ROS2](https://github.com/legubiao/plane_segmentation_ros2) into your workspace src folder, and use rosdep to initialize the dependencies.
```bash
git clone https://github.com/legubiao/plane_segmentation_ros2
```

#### 3.1.1 OCS2 Anymal Models 
* build
```bash
cd ../..
colcon build --packages-up-to ocs2_anymal_models --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
```
* run
```bash
source ../../install/setup.bash
ros2 launch ocs2_anymal_models visualize.launch.py
```