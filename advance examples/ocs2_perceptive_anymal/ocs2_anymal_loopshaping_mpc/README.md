# OCS2 Anymal Loopshaping MPC

This package provided a perceptive mpc demo to allow Anymal_c robot to cross different terrains.

## 1. Build the package

```bash
cd ~/ocs2_ws/
colcon build --packages-up-to ocs2_anymal_loopshaping_mpc 
```

## 2. Perceptive MPC demo

In this launch file, you can tried different terrains.

### 2.1 basic step

```bash
source ~/ocs2_ws/install/setup.bash
ros2 launch ocs2_anymal_loopshaping_mpc perceptive_mpc_demo.launch.py
```

![image-20240806111951428](assets/image-20240806111951428.png)

### 2.2 side gap

```bash
source ~/ocs2_ws/install/setup.bash
ros2 launch ocs2_anymal_loopshaping_mpc perceptive_mpc_demo.launch.py terrain_name:=side_gap.png
```



![image-20240806112246922](assets/image-20240806112246922.png)

### 2.3 gaps

```bash
source ~/ocs2_ws/install/setup.bash
ros2 launch ocs2_anymal_loopshaping_mpc perceptive_mpc_demo.launch.py terrain_name:=gaps.png terrain_scale:=1.0 forward_distance:=7.0
```

![image-20240806112745153](assets/image-20240806112745153.png)

### 2.4 hurdles

```bash
source ~/ocs2_ws/install/setup.bash
ros2 launch ocs2_anymal_loopshaping_mpc perceptive_mpc_demo.launch.py terrain_name:=hurdles.png terrain_scale:=0.7 forward_distance:=7.0
```

![image-20240806113300029](assets/image-20240806113300029.png)

### 2.5 stepping stones

```bash
source ~/ocs2_ws/install/setup.bash
ros2 launch ocs2_anymal_loopshaping_mpc perceptive_mpc_demo.launch.py terrain_name:=stepping_stones.png terrain_scale:=1.0 forward_distance:=7.0
```

![image-20240806111749035](assets/image-20240806111749035.png)