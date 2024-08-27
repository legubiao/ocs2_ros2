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

## Install ONNX Runtime
ONNX Runtime is an inferencing and training accelerator. Here, it is used for deploying learned MPC-Net policies in C++ code. To locally install it, do the following:

* Download the onnxruntime release from Github [onnxruntime 1.19](https://github.com/microsoft/onnxruntime/releases/download/v1.19.0/onnxruntime-linux-x64-1.19.0.tgz)
* Unzip the tgz file, and:
  * Move the `include` folder to `/usr/local/include/onnxruntime`
  * Move the `lib/cmake` folder to `/usr/local/share/cmake`
  * Move the `lib/pkgconfig` folder to `/usr/local/lib/pkgconfig`
  * Move the `lib` to `/usr/local/lib64`
  * ```bash
    sudo ldconfig
    ```
* check the install
  * ```bash
    export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig
    pkg-config --cflags --libs libonnxruntime
    ```
  * The output should be something like:
    
  * `-I/usr/local/include/onnxruntime -L/usr/local/lib64 -lonnxruntime`
  
  

## Build and Run pretrained MPC-Net
```bash
cd ~/ocs2_ws
colcon build --packages-up-to ocs2_ballbot_mpcnet --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

```bash
source ~/ocs2_ws/install/setup.bash
ros2 launch ocs2_ballbot_mpcnet ballbot_mpcnet.launch.py
```

## Create Python Venv

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

## Train MPC-Net
below command assumes that you are in the venv and sourced ros2 workspace.

* train the MPC-Net
```bash
cd ~/ocs2_ws/src/ocs2_ros2/advance\ examples/ocs2_mpcnet/ocs2_mpcnet_core/ocs2_mpcnet_core/
python train.py
```
* check the tensorboard
```bash
cd ~/ocs2_ws/src/ocs2_ros2/advance\ examples/ocs2_mpcnet/ocs2_mpcnet_core/ocs2_mpcnet_core/
tensorboard --logdir=runs
```
to run the trained model, copy the .onnx and .pt file in the runs folder to the policy folder. 