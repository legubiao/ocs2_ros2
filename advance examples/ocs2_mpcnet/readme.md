# OCS2 MPC Net
I just install the latest libraries until 2024.8:
* Ubuntu 24.04
* onnxruntime 1.19
* pybind11 2.13.5
* cuda 12.2


* python 12
* torch 2.4.0
* onnx 1.16.2
* numpy 2.1.0
* black 24.8.0
* pyyaml 6.0.2

Nvidia A500 can train ballbot but cannot train legged robot.

## 1. Install ONNX Runtime
ONNX Runtime is an inferencing and training accelerator. Here, it is used for deploying learned MPC-Net policies in C++ code. To locally install it, do the following:

* Download the onnxruntime release from Github [onnxruntime 1.19](https://github.com/microsoft/onnxruntime/releases/download/v1.19.0/onnxruntime-linux-x64-1.19.0.tgz)
* Unzip the tgz file, and:
  * Move the `include` folder to `/usr/local/include/onnxruntime`
  * Move the `lib/cmake` folder to `/usr/local/share/cmake`
  * Move the `lib/pkgconfig` folder to `/usr/local/lib/pkgconfig`
  * Move the `lib` to `/usr/local/lib64`
  * ```bash
    sudo mv include/ /usr/local/include/onnxruntime/
    sudo mv lib/cmake/ /usr/local/share
    sudo mv lib/pkgconfig/ /usr/local/lib
    sudo mv lib/ /usr/local/lib64
    ```
* check the install
  * ```bash
    export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib64
    
    sudo ldconfig
    pkg-config --cflags --libs libonnxruntime
    ```
  * The output should be something like:
    
  * `-I/usr/local/include/onnxruntime -L/usr/local/lib64 -lonnxruntime`
  
  

## 2. Build and Run pretrained MPC-Net

### 2.1 Ballbot
```bash
cd ~/ocs2_ws
colcon build --packages-up-to ocs2_ballbot_mpcnet
```

```bash
source ~/ocs2_ws/install/setup.bash
ros2 launch ocs2_ballbot_mpcnet ballbot_mpcnet.launch.py
```

### 2.2 Legged Robot
* build the package
```bash
cd ~/ocs2_ws
colcon build --packages-up-to ocs2_legged_robot_mpcnet
```
* launch the simulation without raisim
```bash
source ~/ocs2_ws/install/setup.bash
ros2 launch ocs2_legged_robot_mpcnet default.launch.py
```
* launch the simulation with raisim
```bash
source ~/ocs2_ws/install/setup.bash
ros2 launch ocs2_legged_robot_mpcnet raisim.launch.py
```


## 3. Train MPC-Net
### 3.1 Create Python Venv

* install python3-venv
```bash
sudo apt-get install python3-venv
```

* create a virtual environment
```bash
cd ~
mkdir venvs && cd venvs
python3 -m venv mpcnet
```

* install the required packages
```bash
source ~/venvs/mpcnet/bin/activate
pip install -r ocs2_mpcnet_core/requirements.txt
```
below command assumes that you are in the venv and sourced ros2 workspace.

### 3.2 Ballbot
* train the MPC-Net
```bash
source ~/ocs2_ws/install/setup.bash
cd ~/ocs2_ws/src/ocs2_ros2/advance\ examples/ocs2_mpcnet/ocs2_ballbot_mpcnet/ocs2_ballbot_mpcnet
python train.py
```
* check the tensorboard
```bash
cd ~/ocs2_ws/src/ocs2_ros2/advance\ examples/ocs2_mpcnet/ocs2_ballbot_mpcnet/ocs2_ballbot_mpcnet
tensorboard --logdir=runs
```
to run the trained model, copy the .onnx and .pt file in the runs folder to the policy folder. 

### 3.3 Legged Robot
* train the MPC-Net
```bash
source ~/ocs2_ws/install/setup.bash
cd ~/ocs2_ws/src/ocs2_ros2/advance\ examples/ocs2_mpcnet/ocs2_legged_robot_mpcnet/ocs2_legged_robot_mpcnet
python train.py
```
* train the MPC-Net with the raisim environment
```bash
source ~/ocs2_ws/install/setup.bash
cd ~/ocs2_ws/src/ocs2_ros2/advance\ examples/ocs2_mpcnet/ocs2_legged_robot_mpcnet/ocs2_legged_robot_mpcnet
python train.py legged_robot_raisim.yaml
```
* check the tensorboard
```bash
cd ~/ocs2_ws/src/ocs2_ros2/advance\ examples/ocs2_mpcnet/ocs2_legged_robot_mpcnet/ocs2_legged_robot_mpcnet
tensorboard --logdir=runs
```