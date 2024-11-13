#!/bin/bash

# Install mujoco binaries in the home directory
wget https://mujoco.org/download/mujoco210-linux-x86_64.tar.gz && tar -xvzf mujoco210-linux-x86_64.tar.gz
mkdir -p ~/.mujoco && mv mujoco210 ~/.mujoco/mujoco210
rm -r mujoco210-linux-x86_64.tar.gz

# Install necessaury packages for mujoco-py installation:
sudo apt install libosmesa6-dev libgl1-mesa-glx libglfw3
sudo apt install patchelf
