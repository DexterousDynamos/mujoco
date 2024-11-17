from src.Mujoco_XML import Mujoco_XML
from src.Fusion_Model import Fusion_Model
import numpy as np
from pyquaternion import Quaternion
import os
from typing import Dict, List, Tuple
from dataclasses import dataclass, field
import re
import datetime

@dataclass
class DexterousDynamos:
    # Neccesary inputs
    # /

    # Optional inputs
    model_name: str = "DexterousDynamos"
    asset_folder: str = "assets/"
    json_filename: str = "fusion_info.json"

    # Public variables, not to be set by user
    # /

    # Internal variables
    _env: Mujoco_XML = field(init=False)
    _fusion_data: Fusion_Model = field(init=False)
    # _remapped_fusion_data: Fusion_Model = field(init=False) # Should be removeable in the future
    _quattrans_dict: Dict[str, Tuple[Quaternion, np.ndarray]] = field(init=False, default_factory=dict)
    _joint_transforms: Dict[str, Tuple[np.ndarray, np.ndarray]] = field(init=False, default_factory=dict) # pos, axis # DEBUG - manual joints, should be removed later


    def __post_init__(self):
        # Initialize the Mujoco XML environment
        self._env = Mujoco_XML(model_name=self.model_name)

        # Add assets to the Mujoco XML environment
        self.asset_folder = os.path.abspath(self._find_latest_folder())
        self._add_assets()

        # Read the Fusion JSON file
        self._fusion_data = Fusion_Model(json_file_path=os.path.join(self.asset_folder, self.json_filename))

        # Add components to the Mujoco XML environment
        self._recursive_add_component(self._fusion_data.joint_components[0])

    def _find_latest_folder(self) -> str:
        '''
        Find the latest folder in the asset folder.

        Returns:
            str: The path to the latest folder.
        '''
        base_path = self.asset_folder
        # Regular expression to match folders in the format "fusion_export_YYYY-MM-DD_HH-MM-SS"
        folder_pattern = re.compile(r"fusion_export_(\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})")

        latest_time = None
        latest_folder = None

        # Iterate over items in the base directory
        for folder_name in os.listdir(base_path):
            folder_path = os.path.join(base_path, folder_name)
            
            # Only consider directories that match the naming pattern
            if os.path.isdir(folder_path):
                match = folder_pattern.match(folder_name)
                if match:
                    # Extract and parse the timestamp
                    timestamp_str = match.group(1)
                    folder_time = datetime.datetime.strptime(timestamp_str, "%Y-%m-%d_%H-%M-%S")

                    # Check if this folder is the latest so far
                    if latest_time is None or folder_time > latest_time:
                        latest_time = folder_time
                        latest_folder = folder_path

        return latest_folder

    def _add_assets(self):
        '''
        Add assets to the Mujoco XML environment.
        '''
        for root, _, files in os.walk(self.asset_folder):
            for file in files:
                if file.lower().endswith(".stl"):
                    full_filepath = os.path.abspath(os.path.join(root, file))
                    base_name = os.path.splitext(file)[0]
                    self._env.add_asset(base_name, full_filepath)

    def _recursive_add_component(self, component: Fusion_Model.Component) -> None:
        '''
        Adds a component and its children to the Mujoco XML environment.

        Args:
            component_name (str): The name of the component to add. Its children will also be added.
        '''
        quat, trans = component.relative_transform
        parent_name = component.parent.id if component.parent is not None else ''
        self._env.add_body(component.id, component.stlname, trans, quat, parent_name)

        for child in component.children:
            self._recursive_add_component(child)

    # TODO: Fix correct file path (w.r.t. DexterousDynamos.py, not Mujoco_XML.py)
    def export_xml(self, filename: str):
        '''
        Export the Mujoco XML environment to a file.

        Args:
            filename (str): The name of the file to export to.
        '''
        self._env.export_xml(filename)

    # DEBUG - manual joints
    def calculate_joint_transforms(self):
        '''
        Calculate the joint transforms for the Dexterous Dynamos model.
        '''
        # # tree = {"Root/Carpals:1": "Carpals:1/1"}
        # joint_data ={
        #     # Stuff
        # }
        # for joint in self._fusion_data.joints:
        #     if joint.parent_component.name in self._fusion_data._subassembly_names:
        #         continue
        #     parent_name = joint.parent_component.name
        #     child_name = joint.child_component.name
        #     parent
        pass

    def run_interactive(self):
        '''
        Run the Mujoco simulation interactively.
        '''
        self._env.run_interactive()

if __name__ == "__main__":
    model = DexterousDynamos()
    model.export_xml("xml/model.xml")
    # for comp in model._fusion_data.components:
    #     if comp.parent != None:
    #         print()
    #         print(f"{comp.name}/{comp.id}")
    #         print(f"{comp.parent.name}/{comp.name}")
            
    model.run_interactive()
    exit()
    
    # ############################################ Read out json file ############################################
    # json_file_path = 'assets/fusion_info.json'
    # data = Fusion_Model(json_file_path=json_file_path)
    # data_remapped = Fusion_Model(json_file_path=json_file_path)

    # parent_dict = {"Carpals v54:1": "Root",
    #     "I-Assembly v9:1": "Root", "P-Assembly v82:1": "Root", "P-Assembly v82:2": "Root",
    #     "M-DP v55:1_2": "M_MP v73:1_2", "M_MP v73:1_2": "I_PP v12:1", "I_PP v12:1": "M-AP v67:1_2", "M-AP v67:1_2": "I-Assembly v9:1",
    #     "M-DP v55:1_1": "M_MP v73:1_1", "M_MP v73:1_1": "M_PP v60:1_1", "M_PP v60:1_1": "M-AP v67:1_1", "M-AP v67:1_1": "P-Assembly v82:2",
    #     "M-DP v55:1": "M_MP v73:1", "M_MP v73:1": "M_PP v60:1", "M_PP v60:1": "M-AP v67:1", "M-AP v67:1": "P-Assembly v82:1"}
    # data_remapped.remap_component_tree(parent_dict)

    # ############################################ Create Mujoco XML ############################################
    # env = Mujoco_XML(model_name='main')
    # env.add_default_class("index_joint")
    # env.add_default("joint", "index_joint", axis="1 0 0")

    # for root, _, files in os.walk("assets/"):
    #     for file in files:
    #         if file.lower().endswith(".stl"):
    #             # full_filepath = os.path.join(root, file)
    #             full_filepath = os.path.abspath(os.path.join(root, file))
    #             base_name = os.path.splitext(file)[0].lower()
    #             env.add_asset(base_name, full_filepath)
    # stl_lookup_table = {"Carpals v54:1": "carpals",
    #     "M-AP v67:1_2": "index_0", "I_PP v12:1": "index_1", "M_MP v73:1_2": "index_2", "M-DP v55:1_2": "index_3",
    #     "M-AP v67:1": "middle_0", "M_PP v60:1": "middle_1", "M_MP v73:1": "middle_2", "M-DP v55:1": "middle_3",
    #     "M-AP v67:1_1": "ring_0", "M_PP v60:1_1": "ring_1", "M_MP v73:1_1": "ring_2", "M-DP v55:1_1": "ring_3"}
    # exclude_parent_list = ["I-Assembly v9:1", "P-Assembly v82:1", "P-Assembly v82:2"]
    
    # for child in data.root_component.children:
    #     add_component(env, child.name, data, data_remapped, stl_lookup_table, exclude_parent_list, {})

    # env.export_xml("xml/model.xml")

    # env.run_interactive()
    # # env.run_simulation()