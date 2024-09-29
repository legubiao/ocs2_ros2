# OCS2 Anymal MPC

This package provide a mpc example for Anymal quadruped robot. Besides the basic movement like ocs2_legged_robot_ros, this example provided "demo_motion" example.

![image-20240806094724070](assets/image-20240806094724070.png)

* build command
```bash
cd ~/ocs2_ws/
colcon build --packages-up-to ocs2_anymal_mpc
```

* launch command
```bash
source ~/ocs2_ws/install/setup.bash
ros2 launch ocs2_anymal_mpc anymal_c.launch.py
```