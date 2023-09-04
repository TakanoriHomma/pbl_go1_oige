# pbl_go1_oige
Project Besed Learning - Go1 - Omniverse Isaac Gym Envs

## How to use  
- Go to ```/Docker``` directory and run ```/Docker/build.sh``` file. And then, run ```/run.sh```.

- Automatically you can enter the docker container. Go to ```/isaac-sim/OmniIsaacGymEnvs``` directory.

- Run ```PYTHON_PATH -m pip install -e .``` to install libraries

- Go to ```/isaac-sim/OmniIsaacGymEnvs/omniisaacgymenvs``` directory.

- You can try to train some example environment with ```PYTHON_PATH scripts/rlgames_train_mt.py task={task name. ex)Go1_Horizontal}```. ("mt" means multi-thread.)

- Finally, run ```PYTHON_PATH scripts/rlgames_train.py task={task name} checkpoint={model path. ex)runs/A1_bipedal/nn/A1_bipedal.pth} test=True num_envs={num envs. ex)64}``` to infer the model.
