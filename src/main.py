from Mujoco_XML import Mujoco_XML
from Fusion_Model import Fusion_Model
import numpy as np
from pyquaternion import Quaternion
import os
from typing import Dict, List, Tuple

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

def add_component(env: Mujoco_XML, component_name: str, orig_data: Fusion_Model, remapped_data: Fusion_Model, stl_lookup_table: Dict[str, str], exclude_parent_list: List[str], quattrans_dict: Dict[str, Quaternion]) -> None:
    '''
    Adds a component and its children to the Mujoco XML environment.

    Args:
        env (Mujoco_XML):                         The Mujoco_XML environment to add the component to.
        component_name (str):                     The name of the component to add.
        orig_data (Fusion_Model):                 The original Fusion_Model data.
        remapped_data (Fusion_Model):             The remapped Fusion_Model data.
        stl_lookup_table (Dict[str, str]):        A dictionary mapping component names to STL file names.
        exclude_parent_list (List[str]):          A list of parent names to exclude from the search.
        quattrans_dict (Dict[str, Quaternion]):   A dictionary mapping component names to a list containing the quaternion and translation of the component.
    '''

    def find_joint_parent(component: Fusion_Model.Component, exclude_parent_list: List[str]) -> Fusion_Model.Component:
        '''
        Finds the earliest parent of the component that is not in the exclude_parent_list.

        Args:
            component (Fusion_Model.Component): The component to find the parent of.
            exclude_parent_list (List[str]):    A list of parent names to exclude from the search.

        Returns:
            Fusion_Model.Component:             The parent component.
        '''
        parent = component.parent
        while parent.name in exclude_parent_list:
            parent = parent.parent
        if parent == None:
            raise ValueError("Some error finding parent")
        return parent
    
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
    
    orig_component = orig_data.components[orig_data.find_component_index(component_name)]
    remapped_component = remapped_data.components[remapped_data.find_component_index(component_name)]
    if component_name not in exclude_parent_list:
        quat    = recursive_calculate_quat(orig_component)
        trans   = recursive_calculate_trans(orig_component, orig_component.translation)
        quattrans_dict[component_name] = [quat, trans]

        remapped_parent_name = find_joint_parent(remapped_component, exclude_parent_list).name
        if remapped_parent_name != "Root":
            trans, quat = get_relative_transform(quattrans_dict[remapped_parent_name][1], quattrans_dict[remapped_parent_name][0], trans, quat)
        env.add_body(stl_lookup_table[component_name], pos=trans, quat=quat, parent='' if remapped_parent_name == "Root" else stl_lookup_table[remapped_parent_name])

    for child in remapped_component.children:
        add_component(env, child.name, orig_data, remapped_data, stl_lookup_table, exclude_parent_list, quattrans_dict)

if __name__ == "__main__":
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