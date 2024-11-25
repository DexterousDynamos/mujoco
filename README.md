# Description
This project is a Python package for creating and simulating robotic manipulators in Mujoco using Fusion360 models. Currently, only hinge joints between distinct components are supported.

# Installation instructions
(See `https://github.com/openai/mujoco-py` for most of this. The rest is debugging via online fori)

## Install mujoco binaries
```bash
wget https://mujoco.org/download/mujoco210-linux-x86_64.tar.gz && tar -xvzf mujoco210-linux-x86_64.tar.gz
mkdir -p ~/.mujoco && mv mujoco210 ~/.mujoco/mujoco210
rm -r mujoco210-linux-x86_64.tar.gz
```

## Install pip dependencies
```bash
pip install -r requirements.txt
```

## Add mujoco to the path
```bash
echo -e "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/home/$(whoami)/.mujoco/mujoco210/bin" >> ~/.bashrc
echo -e "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/usr/lib/nvidia" >> ~/.bashrc
echo -e "export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libGLEW.so" >> ~/.bashrc
```

## Install apt dependencies (in case of `-lGL not found` or `patchelf not found` errors)
```bash
sudo apt install libglew-dev libosmesa6-dev libgl1-mesa-glx libglfw3
sudo apt-get install patchelf
```

# Implementation instructions
1) Run the fusion script to create an asset folder (`fusion_export_YYYY-MM-DD_HH-MM-SS`) containing `fusion_info.json` and `.stl` files
2) Copy this folder to the `assets` folder in the root directory of this project.
3) Initialize a `Fusion_to_Mujoco` object in `main.py` with the desired parameters (see `Fusion_to_Mujoco.py` with class definition for defaults).
4) Export the model to a `.xml` file using the `export_to_xml()` method in `main.py`. Optionally, also directly run an interactive window of the model using the `run_interactive()` method.

# TODO
## Fusion360 script
- Add motion links (i.e. linked joints)
- Add windows compatibility

## Mujoco_XML.py
- See if instead of default `<mesh scale="0.001 0.001 0.001">` one can use translations*1000 (in Fusion_Model.py)
- Add checks (incl. list of what kwargs are allowed for each tag)
- (later) Implement "run_simulation" for external commands

## Fusion_Model.py
- See if instead of default `<mesh scale="0.001 0.001 0.001">` (in Mujoco_XML.py) one can use translations*1000
- Add motion links (i.e. equalities - pay attention to remove/not add extra actuator)
- Add checks

## Fusion_to_Mujoco.py
- Add motion links
- Add checks

## main.py
- Add section with all default parameters, for better UI