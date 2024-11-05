import os
import subprocess

from mujoco_py import load_model_from_xml, MjSim, MjViewer
from dataclasses import dataclass, field
from typing import List, Dict
import numpy as np
from pyquaternion import Quaternion # TODO, see https://kieranwynn.github.io/pyquaternion/

@dataclass
class Mujoco_XML:
    # TODO: Add list of what kwargs are allowed for each function
    # TODO: Add checks for valid inputs
    # TODO: Remove not used default tags (e.g. not used contact/actuator)

    # Neccesary inputs
    model_name: str = field()
    use_defaults: str = True

    # Public variables, not to be set by user
    model_str: str = field(init=False)

    # Internal variables
    _first_set: Dict[str, bool] = field(init=False, default_factory=lambda: {
        'compiler': False,
        'default': False,
        'asset': False,
        'worldbody': False,
        'actuator': False,
        'contact': False,
        'collision_class': False,
        'visual_class': False,
    })
    _assets: List[str] = field(init=False, default_factory=list)
    _xml_path: str = field(init=False, default='model.xml')
    _mujoco_sim_path: str = field(init=False, default=os.path.join(os.path.dirname(__file__), 'mujoco_sim.sh'))
    
    def __post_init__(self):
        self.model_str = (
            f'<mujoco model="{self.model_name}">\n'
            '\t<default>\n'
            '\t</default>\n'
            '\t<asset>\n'
            '\t</asset>\n'
            '\t<worldbody>\n'
            '\t</worldbody>\n'
            '\t<actuator>\n'
            '\t</actuator>\n'
            '\t<contact>\n'
            '\t</contact>\n'
            '</mujoco>'
        )

        # Default settings - turn off when creating a new 'Mujoco_XML' object by passing 'use_defaults=False'
        if self.use_defaults:
            self.add_compiler("radian")
            self.add_default_class("collision")
            self.add_default("geom", "collision", contype="1", conaffinity="1", group="4")
            self.add_default_class("visual")
            self.add_default("geom", "visual", contype="0", conaffinity="0", group="1", mass="0")
            # self.add_default("geom", type="mesh", quat=f"{np.sqrt(2)/2} {np.sqrt(2)/2} 0 0")
            self.add_default("geom", type="mesh", quat=f"1 0 0 0") # No Rotation
            self.add_default("position", ctrllimited="true", kp="100")
            self.add_default("velocity", ctrllimited="true", kv="10")
            self.add_default("motor", ctrllimited="true")
            self.add_default("mesh", scale="0.001 0.001 0.001")
            self.add_default("joint", type="hinge", limited="true")
        # self.sim = mujoco_py.MjSim(self.model)

    def insert_at_index(self, target: str, index: int, insert: str, add_end: str = ''):
        # For indent purposes (0 = False, 1 = True)
        if target in self._first_set.keys():
            add_indent = 1 - self._first_set[target]
        else:
            # Target is file name or also contains class name
            add_indent = True
        
        lines = self.model_str.splitlines()
        previous_line = lines[index - 1]
        indent = (len(previous_line) - len(previous_line.lstrip()) + add_indent) * '\t'

        lines.insert(index, indent + insert)
        if add_end != '':
            lines.insert(index + 1, indent + add_end)
        
        self.model_str = '\n'.join(lines)

        if target in self._first_set.keys():
            self._first_set[target] = True

    def insert_after_first(self, target: str, insert: str, add_end: str = ''):
        lines = self.model_str.splitlines()
        # print("Inserting after", target)
        # print("Current xml:", self.model_str)
        target_indices = [i for i, line in enumerate(lines) if target in line]
        self.insert_at_index(target, target_indices[0] + 1, insert, add_end=add_end)

    def insert_before_last(self, target: str, insert: str, add_end: str = ''):
        lines = self.model_str.splitlines()
        target_indices = [i for i, line in enumerate(lines) if target in line]
        self.insert_at_index(target, target_indices[-1], insert, add_end=add_end)

    def add_compiler(self, angle: str = "radian"):
        self.insert_after_first(self.model_name, f'<compiler angle="{angle}"/>')

    def add_default_class(self, class_name: str, parent_class: str = ''):
        if parent_class == '':
            self.insert_before_last("default", f'<default class="{class_name}">', add_end='</default>')
        else:
            self.insert_before_last("default", f'<default class="{class_name}" parent="{parent_class}">', add_end='</default>')

        if class_name == "collision" or class_name == "visual":
            self._first_set[f'{class_name}_class'] = True

    def add_default(self, tag: str, class_name: str = '', **kwargs):
        kwarg_str = ' '.join([f'{key}="{value}"' for key, value in kwargs.items()])
        if class_name == '':
            self.insert_before_last("default", f'<{tag} {kwarg_str}/>')
        else:
            self.insert_after_first(f'default class="{class_name}"', f'<{tag} {kwarg_str}/>')

    def add_asset(self, name: str, filepath: str):
        # Assumes that mesh name is the same as later body name
        self.insert_before_last("asset", f'<mesh name="{name}" file="{filepath}"/>')
        self._assets.append(filepath)

    def add_body(self, name: str, pos: List[float] | np.ndarray = np.array([0, 0, 0]), quat: List[float] | np.ndarray = np.array([1, 0, 0, 0]), parent: str = ''):
        # 'quat' input in format 'w x y z'
        # Assumes that previous mesh name is the same as body name
        if parent == '':
            self.insert_before_last("worldbody", f'<body name="{name}" pos="{pos[0]} {pos[1]} {pos[2]}" quat="{quat[0]} {quat[1]} {quat[2]} {quat[3]}">', add_end='</body>')
        else:
            self.insert_after_first(f'body name="{parent}"', f'<body name="{name}" pos="{pos[0]} {pos[1]} {pos[2]}" quat="{quat[0]} {quat[1]} {quat[2]} {quat[3]}">', add_end='</body>')

        if self._first_set['collision_class']:
            self.insert_after_first(f'body name="{name}"', f'<geom class="collision" mesh="{name}"/>')
        if self._first_set['visual_class']:
            self.insert_after_first(f'body name="{name}"', f'<geom class="visual" mesh="{name}"/>')

    def add_joint(self, body_name: str, pos: List[float] | np.ndarray = np.array([0, 0, 0]), axis: List[float] | np.ndarray = np.array([0, 0, 1]), range: List[float] | np.ndarray = np.array([-1, 1])):
        self.insert_after_first(f'body name="{body_name}"', f'<joint name="joint_{body_name}" pos="{pos[0]} {pos[1]} {pos[2]}" axis="{axis[0]} {axis[1]} {axis[2]}" range="{range[0]} {range[1]}"/>')

    def add_actuator(self, name: str, joint_name: str, actuator_type: str = 'motor', ctrlrange: List[float] | np.ndarray = np.array([-1, 1])):
        # kwarg_str = ' '.join([f'{key}="{value}"' for key, value in kwargs.items()])
        self.insert_before_last("actuator", f'<{actuator_type} name="{name}" joint="{joint_name}" ctrlrange="{ctrlrange[0]} {ctrlrange[1]}"/>')

    def exclude_contact(self, body1: str, body2: str):
        self.insert_before_last("contact", f'<exclude body1="{body1}" body2="{body2}"/>')

    def export_xml(self, filepath: str = 'model.xml'):
        self._xml_path = filepath
        # if filepath == '':
        #     filepath = self._xml_path
        with open(filepath, 'w') as file:
            file.write(self.model_str)

    def run_interactive(self, filepath: str = ''):
        if filepath == '':
            filepath = self._xml_path
        if not os.path.exists(filepath):
            self.export_xml(filepath)
        # Note: Be sure mujoco_sim.sh is in your PATH or in the same directory as this file
        # subprocess.run(["bash", "mujoco_sim.sh"])
        subprocess.run(f"cd {os.path.dirname(filepath)} && bash {self._mujoco_sim_path}", shell=True, check=True)

    def run_simulation(self):
        # Note: Make sure to run the script from its directory
        subprocess.run(["mkdir", "-p", "/tmp/assets"])
        model_str = self.model_str
        for asset in self._assets:
            print("Copying", asset)
            subprocess.run(["cp", asset, "/tmp/assets/"])
            model_str = model_str.replace(asset, f"/tmp/assets/{os.path.basename(asset)}")
                
        model = load_model_from_xml(model_str)
        sim = MjSim(model)
        viewer = MjViewer(sim)
        while True:
            viewer.render()

if __name__ == "__main__":
    env = Mujoco_XML(model_name='exclude_main_template.xml')
    env.add_default_class("index_joint")
    env.add_default("joint", "index_joint", axis="1 0 0")

    env.add_asset("carpals", "assets/Carpals.stl")
    env.add_asset("middle_0", "assets/Middle/Middle_0.stl")

    env.add_body("carpals", pos=[0, 0, -0.5])
    env.add_body("middle_0", pos=[0.003436, 0.000077, 0.083892], parent="carpals")

    env.add_joint("middle_0", pos=[0.003436, 0.000077, 0.083892], axis=[0, -1, 0], range=[-0.78, 0.78])

    env.add_actuator("motor_joint", "joint_middle_0")

    env.exclude_contact("carpals", "middle_0")

    env.export_xml()

    # print(env.model_str)
    env.run_interactive()
    # env.run_simulation()