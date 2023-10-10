#!/bin/bash

PARENT_DIR=$(pwd|xargs dirname)

# docker run \
#   --net=host \
#   --gpus all \
#   -v $HOME/.Xauthority:/root/.Xauthority:rw \
#   -v /tmp/.X11-unix:/tmp/.X11-unix \
#   -v /etc/localtime:/etc/localtime:ro \
#   -v /usr/share/vulkan/icd.d/nvidia_icd.json:/etc/vulkan/icd.d/nvidia_icd.json \
#   -v /usr/share/vulkan/implicit_layer.d/nvidia_layers.json:/etc/vulkan/implicit_layer.d/nvidia_layers.json \
#   -v /usr/share/glvnd/egl_vendor.d/10_nvidia.json:/usr/share/glvnd/egl_vendor.d/10_nvidia.json \
#   -v ~/Documents/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
#   -v ~/Documents/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
#   -v ~/Documents/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
#   -v ~/Documents/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
#   -v ~/Documents/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
#   -v ~/Documents/docker/isaac-sim/config:/root/.nvidia-omniverse/config:rw \
#   -v ~/Documents/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
#   -v ~/Documents/docker/isaac-sim/documents:/root/Documents:rw \
#   -v ~/Documents/Quadruped_bipedal/isaacgym-env:/isaac-sim/isaacgym-env:rw \
#   -v ~/Documents/Quadruped_bipedal/OmniIsaacGymEnvs:/isaac-sim/OmniIsaacGymEnvs:rw \
#   -e ACCEPT_EULA=Y \
#   -e DISPLAY=$DISPLAY \
#   -e NVIDIA_VISIBLE_DEVICES=all \
#   -e NVIDIA_DRIVER_CAPABILITIES=all \
#   -it --rm --name "isaacgym-docker" isaacgym-docker:2022.2.0

docker run \
  --net=host \
  --gpus all \
  -v $HOME/.Xauthority:/root/.Xauthority:rw \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /etc/localtime:/etc/localtime:ro \
  -v /usr/share/vulkan/icd.d/nvidia_icd.json:/etc/vulkan/icd.d/nvidia_icd.json \
  -v /usr/share/vulkan/implicit_layer.d/nvidia_layers.json:/etc/vulkan/implicit_layer.d/nvidia_layers.json \
  -v /usr/share/glvnd/egl_vendor.d/10_nvidia.json:/usr/share/glvnd/egl_vendor.d/10_nvidia.json \
  -v "${PARENT_DIR}"/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
  -v "${PARENT_DIR}"/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
  -v "${PARENT_DIR}"/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
  -v "${PARENT_DIR}"/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
  -v "${PARENT_DIR}"/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
  -v "${PARENT_DIR}"/docker/isaac-sim/config:/root/.nvidia-omniverse/config:rw \
  -v "${PARENT_DIR}"/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
  -v "${PARENT_DIR}"/docker/isaac-sim/documents:/root/Documents:rw \
  -v "${PARENT_DIR}"/isaacgym-env:/isaac-sim/isaacgym-env:rw \
  -v "${PARENT_DIR}"/OmniIsaacGymEnvs:/isaac-sim/OmniIsaacGymEnvs:rw \
  -e ACCEPT_EULA=Y \
  -e DISPLAY=$DISPLAY \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -it --name "go1-isaacgym-docker" go1-isaacgym-docker:2023.9.1
