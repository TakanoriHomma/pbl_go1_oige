# pbl_go1_oige
Project Besed Learning - Go1 - Omniverse Isaac Gym Envs

## How to use  
- Please check the following site and run all commands to install NVIDIA Container Toolkit and generate NGC API Key.  
https://docs.omniverse.nvidia.com/isaacsim/latest/install_container.html

- Go to ```/Docker``` directory and run ```/Docker/build.sh``` file. And then, run ```/run.sh```.

- Automatically you can enter the docker container. Go to ```/isaac-sim/OmniIsaacGymEnvs``` directory.

- Run ```PYTHON_PATH -m pip install -e .``` to install libraries

- Go to ```/isaac-sim/OmniIsaacGymEnvs/omniisaacgymenvs``` directory.

- You can try to train some example environment with ```PYTHON_PATH scripts/rlgames_train_mt.py task=Go1_Horizontal```. ("mt" means multi-thread.)

- Finally, run ```PYTHON_PATH scripts/rlgames_train.py task=Go1_Horizontal checkpoint=runs/A1_bipedal/nn/Go1/Go1.pth test=True num_envs=1``` to infer the model.

## About ROS
```bash
sudo apt install ros-noetic-move-base-msgs
sudo apt install liblcm-dev

cd /Isaacsim/OmniIsaacGymEnvs/unitree_ws/src/unitree_legged_sdk/build
cmake ..
make

cd /Isaacsim/OmniIsaacGymEnvs/unitree_ws
catkin_make

cd /Isaacsim/OmniIsaacGymEnvs/unitree_ws
source devel/setup.bash
roslaunch unitree_guide gazeboSim.launch


# TO LAUNCH EXAMPLE
#(in other terminal)
cd ~/unitree_ws
source devel/setup.bash
./devel/lib/unitree_guide/junior_ctrl

```
