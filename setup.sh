#!/bin/bash

# Activate virtual environment
source mujoco_env/bin/activate

# Set environment variables
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/$(whoami)/.mujoco/mujoco210/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/nvidia
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libGLEW.so
