## Install ONNX Runtime
ONNX Runtime is an inferencing and training accelerator. Here, it is used for deploying learned MPC-Net policies in C++ code. To locally install it, do the following:

```bash
cd /tmp
wget https://github.com/microsoft/onnxruntime/releases/download/v1.7.0/onnxruntime-linux-x64-1.7.0.tgz
tar xf onnxruntime-linux-x64-1.7.0.tgz
sudo mkdir -p /usr/local/bin /usr/local/include/onnxruntime /usr/local/lib /usr/local/share/cmake/onnxruntime
sudo rsync -a /tmp/onnxruntime-linux-x64-1.7.0/include/ /usr/local/include/onnxruntime
sudo rsync -a /tmp/onnxruntime-linux-x64-1.7.0/lib/ /usr/local/lib
sudo rsync -a ~/ocs2_ws/src/ocs2_ros2/advance\ examples/ocs2_mpcnet/ocs2_mpcnet_core/misc/onnxruntime/cmake/ /usr/local/share/cmake/onnxruntime

sudo ldconfig
```

## Build and Run pretrained MPC-Net
```bash
cd ~/ocs2_ws
colcon build --packages-up-to ocs2_ballbot_mpcnet --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

## Create a python venv

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