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
    _quattrans_dict: Dict[str, List[Quaternion, np.ndarray]] = field(init=False, default_factory=dict)


    def __post_init__(self):
        # Initialize the Mujoco XML environment
        self._env = Mujoco_XML(model_name=self.model_name)

        # Add assets to the Mujoco XML environment
        self.asset_folder = os.path.abspath(self._find_latest_folder())
        self._add_assets()

        # Read the Fusion JSON file
        self._fusion_data = Fusion_Model(json_file_path=os.path.join(self.asset_folder, self.json_filename))

        # Add components to the Mujoco XML environment
        for child in self._fusion_data.root_component.children:
            self.add_component(child.name)

    def _find_latest_folder(self):
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
                    base_name = os.path.splitext(file)[0] # .lower()
                    self._env.add_asset(base_name, full_filepath)

    def _compname_to_stlname(self, comp_name: str):
        '''
        Convert a Fusion component name to an STL file name.

        Args:
            comp_name (str): The Fusion component name.

        Returns:
            str: The STL file name.
        '''
        split_name = comp_name.split(":")
        if len(split_name) > 2:
            raise ValueError("Component name should not contain more than one colon")
        elif len(split_name) == 1:
            raise ValueError("Component name should contain a colon")
        return split_name[0]

    def find_joint_parent(self, component: Fusion_Model.Component) -> Fusion_Model.Component:
        '''
        Finds the earliest parent of the component that is not in the exclude_parent_list.

        Args:
            component (Fusion_Model.Component): The component to find the parent of.

        Returns:
            Fusion_Model.Component:             The parent component.
        '''
        parent = component.parent
        while parent.name in self._fusion_data._subassembly_names:
            parent = parent.parent
        if parent == None:
            raise ValueError("Some error finding parent")
        return parent

    def add_component(self, component_name: str) -> None:
        '''
        Adds a component and its children to the Mujoco XML environment.

        Args:
            component_name (str): The name of the component to add. Its children will also be added.
        '''

        def get_relative_transform(parent_pos: np.ndarray, parent_quat: np.ndarray | Quaternion, child_pos: np.ndarray, child_quat: np.ndarray | Quaternion) -> Tuple[np.ndarray, np.ndarray | Quaternion]:
            """
            Calculate the relative position and quaternion of a child body relative to its parent.
            
            Args:
                parent_pos (np.ndarray):                  Position of the parent body as a numpy array.
                parent_quat (np.ndarray | Quaternion):    Quaternion of the parent body as a numpy array (w, x, y, z).
                child_pos (np.ndarray):                   Position of the child body as a numpy array.
                child_quat (np.ndarray | Quaternion):     Quaternion of the child body as a numpy array (w, x, y, z).
            
            Returns:
                (Tuple[np.ndarray, np.ndarray | Quaternion]): The relative position and quaternion of the child body.
            """
            if isinstance(parent_quat, np.ndarray):
                parent_quat = Quaternion(parent_quat).normalised
                parent_is_np = True
            else:
                if not np.isclose(parent_quat.norm, 1):
                    parent_quat = parent_quat.normalised
                parent_is_np = False

            if isinstance(child_quat, np.ndarray):
                child_quat = Quaternion(child_quat).normalised
                child_is_np = True
            else:
                if not np.isclose(child_quat.norm, 1):
                    child_quat = child_quat.normalised
                child_is_np = False

            # Step 1: Calculate relative position in the parent's frame
            # Translate child position by the inverse of the parent's rotation
            relative_pos = parent_quat.inverse.rotate(child_pos - parent_pos)

            # Step 2: Calculate relative orientation (quaternion)
            relative_quat = parent_quat.inverse * child_quat

            if parent_is_np and child_is_np:
                relative_quat = relative_quat.normalised.elements
            
            return relative_pos, relative_quat
        
        def recursive_calculate_quat(component: Fusion_Model.Component) -> Quaternion:
            '''
            Recursively calculates the total quaternion of the component.

            Args:
                component (Fusion_Model.Component): The component to calculate the quaternion of.

            Returns:
                Quaternion: The total quaternion of the component.
            '''
            if component.name == "Root":
                return component.quaternion
            return recursive_calculate_quat(component.parent) * component.quaternion
        
        def recursive_calculate_trans(component: Fusion_Model.Component, component_translation: np.ndarray) -> np.ndarray:
            '''
            Recursively calculates the total translation of the component.

            Args:
                component (Fusion_Model.Component): The component to calculate the translation of.
                component_translation (np.ndarray): The translation of the component.

            Returns:
                np.ndarray: The total translation of the component.
            '''
            if component.name == "Root":
                return component_translation
            return recursive_calculate_trans(component.parent, component.parent.quaternion.rotate(component_translation) + component.parent.translation)
        
        # orig_component = orig_data.components[orig_data.find_component_index(component_name)]
        # remapped_component = remapped_data.components[remapped_data.find_component_index(component_name)]
        # if component_name not in exclude_parent_list:
        #     quat    = recursive_calculate_quat(orig_component)
        #     trans   = recursive_calculate_trans(orig_component, orig_component.translation)
        #     quattrans_dict[component_name] = [quat, trans]

        #     remapped_parent_name = self.find_joint_parent(remapped_component, exclude_parent_list).name
        #     if remapped_parent_name != "Root":
        #         trans, quat = get_relative_transform(quattrans_dict[remapped_parent_name][1], quattrans_dict[remapped_parent_name][0], trans, quat)
        #     self.env.add_body(stl_lookup_table[component_name], pos=trans, quat=quat, parent='' if remapped_parent_name == "Root" else stl_lookup_table[remapped_parent_name])

        # for child in remapped_component.children:
        #     self.add_component(child.name, orig_data, remapped_data, stl_lookup_table, exclude_parent_list, quattrans_dict)

        orig_component = self._fusion_data.components[self._fusion_data.find_component_index(component_name)]
        remapped_component = self._fusion_data.joint_components[self._fusion_data.find_joint_component_index(component_name)]
        if component_name not in self._fusion_data._subassembly_names:
            quat    = recursive_calculate_quat(orig_component)
            trans   = recursive_calculate_trans(orig_component, orig_component.translation)
            self._quattrans_dict[component_name] = [quat, trans]

            remapped_parent_name = self.find_joint_parent(remapped_component).name
            if remapped_parent_name != "Root":
                trans, quat = get_relative_transform(self._quattrans_dict[remapped_parent_name][1], self._quattrans_dict[remapped_parent_name][0], trans, quat)
            self._env.add_body(self._compname_to_stlname(component_name), pos=trans, quat=quat, parent='' if remapped_parent_name == "Root" else self._compname_to_stlname(remapped_parent_name))

        for child in remapped_component.children:
            self.add_component(child.name)

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

    a = DexterousDynamos()
    exit()
    ############################################ Read out json file ############################################
    json_file_path = 'assets/fusion_info.json'
    data = Fusion_Model(json_file_path=json_file_path)
    data_remapped = Fusion_Model(json_file_path=json_file_path)

    parent_dict = {"Carpals v54:1": "Root",
        "I-Assembly v9:1": "Root", "P-Assembly v82:1": "Root", "P-Assembly v82:2": "Root",
        "M-DP v55:1_2": "M_MP v73:1_2", "M_MP v73:1_2": "I_PP v12:1", "I_PP v12:1": "M-AP v67:1_2", "M-AP v67:1_2": "I-Assembly v9:1",
        "M-DP v55:1_1": "M_MP v73:1_1", "M_MP v73:1_1": "M_PP v60:1_1", "M_PP v60:1_1": "M-AP v67:1_1", "M-AP v67:1_1": "P-Assembly v82:2",
        "M-DP v55:1": "M_MP v73:1", "M_MP v73:1": "M_PP v60:1", "M_PP v60:1": "M-AP v67:1", "M-AP v67:1": "P-Assembly v82:1"}
    data_remapped.remap_component_tree(parent_dict)

    ############################################ Create Mujoco XML ############################################
    env = Mujoco_XML(model_name='main')
    env.add_default_class("index_joint")
    env.add_default("joint", "index_joint", axis="1 0 0")

    for root, _, files in os.walk("assets/"):
        for file in files:
            if file.lower().endswith(".stl"):
                # full_filepath = os.path.join(root, file)
                full_filepath = os.path.abspath(os.path.join(root, file))
                base_name = os.path.splitext(file)[0].lower()
                env.add_asset(base_name, full_filepath)
    stl_lookup_table = {"Carpals v54:1": "carpals",
        "M-AP v67:1_2": "index_0", "I_PP v12:1": "index_1", "M_MP v73:1_2": "index_2", "M-DP v55:1_2": "index_3",
        "M-AP v67:1": "middle_0", "M_PP v60:1": "middle_1", "M_MP v73:1": "middle_2", "M-DP v55:1": "middle_3",
        "M-AP v67:1_1": "ring_0", "M_PP v60:1_1": "ring_1", "M_MP v73:1_1": "ring_2", "M-DP v55:1_1": "ring_3"}
    exclude_parent_list = ["I-Assembly v9:1", "P-Assembly v82:1", "P-Assembly v82:2"]
    
    for child in data.root_component.children:
        add_component(env, child.name, data, data_remapped, stl_lookup_table, exclude_parent_list, {})

    env.export_xml("xml/model.xml")

    env.run_interactive()
    # env.run_simulation()