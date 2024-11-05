# Installation instructions
(See `https://github.com/openai/mujoco-py` for most of this. The rest is debugging via online fori)
## Install pip dependencies
```bash
pip install -U 'mujoco-py<2.2,>=2.1' mujoco cython
```

## Install mujoco
```bash
wget https://mujoco.org/download/mujoco210-linux-x86_64.tar.gz && tar -xvzf mujoco210-linux-x86_64.tar.gz
mkdir -p ~/.mujoco && mv mujoco210 ~/.mujoco/mujoco210
rm -r mujoco210-linux-x86_64.tar.gz
```

## Add mujoco to the path
```bash
echo -e "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/$(whoami)/.mujoco/mujoco210/bin" >> ~/.bashrc
echo -e "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/nvidia" >> ~/.bashrc
echo -e "export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libGLEW.so" >> ~/.bashrc
```

## Install apt dependencies (in case of `-lGL not found` or `patchelf not found` errors)
```bash
sudo apt install libosmesa6-dev libgl1-mesa-glx libglfw3
sudo apt-get install patchelf
```

# TODO
Will add more instructions later, when I have time.
