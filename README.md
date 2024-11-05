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
pip install -U 'mujoco-py<2.2,>=2.1' mujoco cython
```

## Add mujoco to the path
```bash
echo -e "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/arturo/.mujoco/mujoco210/bin" >> ~/.bashrc
echo -e "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/nvidia" >> ~/.bashrc
echo -e "export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libGLEW.so" >> ~/.bashrc
```

## Install apt dependencies (in case of `-lGL not found` or `patchelf not found` errors)
```bash
sudo apt install libosmesa6-dev libgl1-mesa-glx libglfw3
sudo apt-get install patchelf
```

# Implementation instructions
Create a model via `main.py`. This will create a mujoco `.xml` file in the `xml` folder (overwrites if already existent). This file can be used to run an interarctive simulation via using the `run_interactive()` method (which simply sources the `mujoco_sim.sh` script). The script excludes any `.xml` files in the `xml` folder whose name starts with `exclude`.

For reading automated Fusion360 files (created using the python script for fusion `TODO`), use the `Fusion_Model` class (see `Fusion_Model.py` for usage example). For creating the string later used to export to a `.xml` file, use the `Mujoco_XML` class (see `Mujoco_XML.py` for usage example). Both of these classes are combined in `main.py` for creating an automated mujoco model based on Fusion360 exported data.

# TODO
## General
- Summarize pip dependencies in a requirements.txt file
- Add 

## Fusion360 script
- Add correct dimensions to fusion script (translation /= 100)
- automate stl export
- automate (for main.py) "exclude_parent_list" (sees which files are an assembly and which are components; i.e. assemblies have more than 1 component)
- automate asset folder and script to github
- (if possible) automate the creation of the joint axis (should be doable via the joint axis in the original file)
- (if possible) automate the creation of the joint limits
- (if possible) automate the creation remapped_data (i.e. which component is joned to which component -> parent_dict (in main.py))

## Mujoco_XML.py
- Add checks
- Add list of what kwargs are allowed for each tag
- (search for "TODO" in file) "add_body" add separate mesh name from body name
- (search for "TODO" in file) perfect/debug "add_joint", "add_actuator", "exclude_contact" (i.e. test via main.py)
- (later) Implement "run_simulation" for external commands

Less important:
- (not important; search for "TODO" in file) "add_default_class" add parent class
- (not necessary for mujoco; maybe for isaac gym) Remove not used default-created tags (e.g. not used contact/actuator)

## Fusion_Model.py
- Add checks
- (search for "TODO" in file) multi-assembly-layer fix in "_build_component_tree"

## main.py
- ! Implement joints and actuators !
- Add checks
- Cleanup via "DexterousDynamosHand" class (maybe via new file)
- User defined "joint list", which simplifies "parent_dict"
- Simply input joint axis (w.r.t. original file), then rotate via quaternion (also a good check to see if two bodies have correct quaternion)