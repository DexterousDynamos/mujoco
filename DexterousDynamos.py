from src.Mujoco_XML import Mujoco_XML
from src.Fusion_Model import Fusion_Model
import numpy as np
from pyquaternion import Quaternion
import os
from typing import Dict, List, Tuple
from dataclasses import dataclass, field
import re
import datetime
from src.utils import reduce_mesh, find_latest_folder

@dataclass
class DexterousDynamos:
    # Neccesary inputs
    # /

    # Optional inputs
    model_name:         str = "DexterousDynamos"
    asset_folder:       str = "assets/"
    json_filename:      str = "fusion_info.json"
    reduction_level:    str = None

    # Public variables, not to be set by user
    # /

    # Internal variables
    _env: Mujoco_XML = field(init=False)
    _fusion_data: Fusion_Model = field(init=False)

    def __post_init__(self):
        # Initialize the Mujoco XML environment
        self._env = Mujoco_XML(model_name=self.model_name)

        # Add assets to the Mujoco XML environment
        # if self.reduction_level is not None: # TODO: Implement this
        #     reduce_mesh(self.asset_folder, self.reduction_level)
        self.asset_folder = os.path.abspath(find_latest_folder(self.asset_folder))
        self._add_assets()

        # Read the Fusion JSON file
        self._fusion_data = Fusion_Model(json_file_path=os.path.join(self.asset_folder, self.json_filename))

        # Add components to the Mujoco XML environment
        self._recursive_add_component(self._fusion_data.joint_components[0])

    # def _find_latest_folder(self) -> str:
    #     '''
    #     Find the latest folder in the asset folder.

    #     Returns:
    #         str: The path to the latest folder.
    #     '''
    #     base_path = self.asset_folder
    #     # Regular expression to match folders in the format "fusion_export_YYYY-MM-DD_HH-MM-SS"
    #     folder_pattern = re.compile(r"fusion_export_(\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})")

    #     latest_time = None
    #     latest_folder = None

    #     # Iterate over items in the base directory
    #     for folder_name in os.listdir(base_path):
    #         folder_path = os.path.join(base_path, folder_name)
            
    #         # Only consider directories that match the naming pattern
    #         if os.path.isdir(folder_path):
    #             match = folder_pattern.match(folder_name)
    #             if match:
    #                 # Extract and parse the timestamp
    #                 timestamp_str = match.group(1)
    #                 folder_time = datetime.datetime.strptime(timestamp_str, "%Y-%m-%d_%H-%M-%S")

    #                 # Check if this folder is the latest so far
    #                 if latest_time is None or folder_time > latest_time:
    #                     latest_time = folder_time
    #                     latest_folder = folder_path

    #     return latest_folder

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
        if component.joint is not None:
            self._env.add_joint(component.id, component.joint.joint_name, component.joint.relative_transform[1], component.joint.relative_transform[0], component.joint.range)
            self._env.add_actuator(component.joint.joint_name + "_actuator", component.joint.joint_name, ctrlrange=component.joint.range)

        for child in component.children:
            self._recursive_add_component(child)

    def export_xml(self, filename: str):
        '''
        Export the Mujoco XML environment to a file.

        Args:
            filename (str): The name of the file to export to.
        '''
        self._env.export_xml(filename)

    def run_interactive(self):
        '''
        Run the Mujoco simulation interactively.
        '''
        self._env.run_interactive()

if __name__ == "__main__":
    model = DexterousDynamos()
    model.export_xml("xml/model.xml")
    model.run_interactive()