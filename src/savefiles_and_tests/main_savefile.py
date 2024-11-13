from Mujoco_XML import Mujoco_XML
from Fusion_Model import Fusion_Model
import numpy as np
from pyquaternion import Quaternion # TODO, see https://kieranwynn.github.io/pyquaternion/
# import json
import os
from typing import Dict, List

# TODO: Do connections via "Joint list" (define joint between b1 ,b2 etc. and from that automatically find hierarchy)
# TODO: Simply input joint axis in original file, then rotate via quaternion (also a good check to see if two bodies have correct quaternion)
# CCC: Component ID, joint vectors, make new assembly file (but keep old one)
# TODO: Automate stl export

def quaternion_mult(q2, q1):
    # Extract individual components
    w2, x2, y2, z2 = q2
    w1, x1, y1, z1 = q1
    
    # Perform the quaternion multiplication
    w = w2 * w1 - x2 * x1 - y2 * y1 - z2 * z1
    x = w2 * x1 + x2 * w1 + y2 * z1 - z2 * y1
    y = w2 * y1 - x2 * z1 + y2 * w1 + z2 * x1
    z = w2 * z1 + x2 * y1 - y2 * x1 + z2 * w1
    
    return np.array([w, x, y, z])

def apply_quaternion(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    def quaternion_conjugate(q):
        # Conjugate of a quaternion [w, x, y, z] is [w, -x, -y, -z]
        return np.array([q[0], -q[1], -q[2], -q[3]])

    # Convert vector to a quaternion with 0 scalar part
    v_quat = np.insert(v, 0, 0)
    
    # Calculate q * v * q_conjugate
    q_conjugate = quaternion_conjugate(q)
    rotated_quat = quaternion_mult(quaternion_mult(q, v_quat), q_conjugate)
    
    # Return the vector part of the resulting quaternion
    return rotated_quat[1:]

# def add_component(env: Mujoco_XML, component: Fusion_Model.Component, lookup_table: Dict[str, str] = {}):
#     if component.name == "Carpals v54:1":
#         env.add_body(lookup_table[component.name], pos=component.translation, parent=lookup_table[component.parent.name] if component.parent.name is not "Root" else '', quat=component.quaternion)
#         # env.add_joint(lookup_table[component.name], pos=component.translation, axis=component.rotation_matrix[:, 2], range=[-0.78, 0.78])
#         # env.add_actuator(f"motor_{lookup_table[component.name]}", f"joint_{lookup_table[component.name]}")
#         # env.exclude_contact("carpals", lookup_table[component.name])
#     if component.name == "M-AP v67:1_2":
#         parent = component.parent
#         quat = quaternion_mult(parent.quaternion, component.quaternion)
#         trans = apply_quaternion(parent.quaternion, component.translation) + parent.translation
#         env.add_body(lookup_table[component.name], pos=trans, quat=quat)

#     for child in component.children:
#         add_component(env, child, lookup_table)


# def add_component(env: Mujoco_XML, component: Fusion_Model.Component, stl_lookup_table: Dict[str, str], exclude_parent_list: List[str]):
#     # Finds the earliest parent of the component that is not in the exclude_parent_list
#     def find_joint_parent(component: Fusion_Model.Component, exclude_parent_list: List[str]) -> Fusion_Model.Component:
#         parent = component.parent
#         while parent.name in exclude_parent_list:
#             parent = parent.parent
#         if parent == None:
#             raise ValueError("Some error finding parent")
#         return parent
    
#     # Calculates the total quaternion of the component
#     def recursive_calculate_quat(component: Fusion_Model.Component) -> np.ndarray:
#         if component.name == "Root":
#             return component.quaternion
#         return quaternion_mult(recursive_calculate_quat(component.parent), component.quaternion)
    
#     # Calculates the total translation of the component
#     def recursive_calculate_trans(component: Fusion_Model.Component, component_translation: np.ndarray) -> np.ndarray:
#         if component.name == "Root":
#             return component_translation
#         return recursive_calculate_trans(component.parent, apply_quaternion(component.parent.quaternion, component_translation) + component.parent.translation)
    

#     if component.name not in exclude_parent_list:
#         parent_name = find_joint_parent(component, exclude_parent_list).name
#         # parent_name = parent_dict[component.name]
#         # parent_name = component.parent.name
#         # print(f"\nAdding {component.name} with parent {parent_name}")
#         # print(f"Component {component.name} equivalent to {stl_lookup_table[component.name]}")
#         # if parent_name == "Root":
#         #     print(f"Parent {parent_name}")
#         # else:
#         #     print(f"Parent {parent_name} equivalent to {stl_lookup_table[parent_name]}")
#         quat = recursive_calculate_quat(component)
#         # if "Carpals" not in component.name:
#         #     quat = quaternion_mult(quat, np.array([1, 1, 0, 0])*np.sqrt(2)/2) # [w, x, y, z], Fusion360 to Mujoco
#         print(f"Quaternion for {component.name}:", np.around(quat, 3))
#         trans = recursive_calculate_trans(component, component.translation)
#         print(f"Translation for {component.name}: ", np.around(trans, 3))
#         # env.add_body(stl_lookup_table[component.name], pos=trans, quat=quat, parent=parent_name)
#         env.add_body(stl_lookup_table[component.name], pos=trans, quat=quat, parent='' if parent_name == "Root" else stl_lookup_table[parent_name])

#     for child in component.children:
#         add_component(env, child, stl_lookup_table, exclude_parent_list)


def add_component(env: Mujoco_XML, component_name: str, orig_data: Fusion_Model, remapped_data: Fusion_Model, stl_lookup_table: Dict[str, str], exclude_parent_list: List[str]):
    # Finds the earliest parent of the component that is not in the exclude_parent_list
    def find_joint_parent(component: Fusion_Model.Component, exclude_parent_list: List[str]) -> Fusion_Model.Component:
        parent = component.parent
        while parent.name in exclude_parent_list:
            parent = parent.parent
        if parent == None:
            raise ValueError("Some error finding parent")
        return parent
    
    # Calculates the total quaternion of the component
    def recursive_calculate_quat(component: Fusion_Model.Component) -> np.ndarray:
        if component.name == "Root":
            return component.quaternion
        return quaternion_mult(recursive_calculate_quat(component.parent), component.quaternion)
    
    # Calculates the total translation of the component
    def recursive_calculate_trans(component: Fusion_Model.Component, component_translation: np.ndarray) -> np.ndarray:
        if component.name == "Root":
            return component_translation
        return recursive_calculate_trans(component.parent, apply_quaternion(component.parent.quaternion, component_translation) + component.parent.translation)
    
    orig_component = orig_data.components[orig_data.find_component_index(component_name)]
    remapped_component = remapped_data.components[remapped_data.find_component_index(component_name)]
    if component_name not in exclude_parent_list:
        parent_name = find_joint_parent(orig_component, exclude_parent_list).name
        # parent_name = parent_dict[component.name]
        # parent_name = component.parent.name
        # print(f"\nAdding {component.name} with parent {parent_name}")
        # print(f"Component {component.name} equivalent to {stl_lookup_table[component.name]}")
        if parent_name == "Root":
            # print(f"Parent {parent_name}")
            # print(type(component_name))
            # print(type(stl_lookup_table))
            stl_lookup_table[component_name]
        else:
            print(f"Parent {parent_name} equivalent to {stl_lookup_table[parent_name]}")
        quat = recursive_calculate_quat(orig_component)
        # if "Carpals" not in component.name:
        #     quat = quaternion_mult(quat, np.array([1, 1, 0, 0])*np.sqrt(2)/2) # [w, x, y, z], Fusion360 to Mujoco
        # print(f"Quaternion for {component.name}:", np.around(quat, 3))
        trans = recursive_calculate_trans(orig_component, orig_component.translation)
        # print(f"Translation for {component.name}: ", np.around(trans, 3))
        # env.add_body(stl_lookup_table[component.name], pos=trans, quat=quat, parent=parent_name)
        env.add_body(stl_lookup_table[component_name], pos=trans, quat=quat, parent='' if parent_name == "Root" else stl_lookup_table[parent_name])

    for child in remapped_component.children:
        add_component(env, child.name, orig_data, remapped_data, stl_lookup_table, exclude_parent_list)


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
    # print(data)
    # data.remap_component_tree(parent_dict)
    print(data)
    print(data_remapped)
    # exit()
    # exit()
    # print(data.root_component.name)
    # print([data.root_component.children[i].name for i in range(len(data.root_component.children))])

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
    # env.add_asset("carpals", "assets/Carpals.stl")
    # env.add_asset("middle_0", "assets/Middle/Middle_0.stl")
    stl_lookup_table = {"Carpals v54:1": "carpals",
        "M-AP v67:1_2": "index_0", "I_PP v12:1": "index_1", "M_MP v73:1_2": "index_2", "M-DP v55:1_2": "index_3",
        "M-AP v67:1": "middle_0", "M_PP v60:1": "middle_1", "M_MP v73:1": "middle_2", "M-DP v55:1": "middle_3",
        "M-AP v67:1_1": "ring_0", "M_PP v60:1_1": "ring_1", "M_MP v73:1_1": "ring_2", "M-DP v55:1_1": "ring_3"}
    exclude_parent_list = ["I-Assembly v9:1", "P-Assembly v82:1", "P-Assembly v82:2"]
    
    # base_quat = np.array([1, 0, 0, 1])*np.sqrt(2)/2 # [w, x, y, z], Fusion360 to Mujoco
    # print(data.root_component.quaternion)
    # print(data.root_component.parent)
    for child in data.root_component.children:
        # add_component(env, child, stl_lookup_table, exclude_parent_list)
        add_component(env, child.name, data, data_remapped, stl_lookup_table, exclude_parent_list)
        # add_component(env, child, stl_lookup_table)

    # base_quat = np.array([0, 0, 0, 1])
    # print(base_quat)
    # base_quat = np.array([1, 0, 0, 0])*np.sqrt(2)/2 # [w, x, y, z], Fusion360 to Mujoco
    # for component in data.components:
    #     if component.name == "Carpals v54:1":
    #         env.add_body("carpals", pos=component.translation, quat=component.quaternion) #, quat=quaternion_mult(component.quaternion, base_quat))
        # if component.name in lookup_table:
        #     env.add_body(lookup_table[component.name], pos=component.translation, parent=lookup_table[component.parent.name] if component.parent is not None else None)
        #     env.add_joint(lookup_table[component.name], pos=component.translation, axis=component.rotation_matrix[:, 2], range=[-0.78, 0.78])
        #     env.add_actuator(f"motor_{lookup_table[component.name]}", f"joint_{lookup_table[component.name]}")
        #     env.exclude_contact("carpals", lookup_table[component.name])

    # env.add_body("carpals", pos=[0, 0, -0.5])
    # env.add_body("middle_0", pos=[0.003436, 0.000077, 0.083892], parent="carpals")

    # env.add_joint("middle_0", pos=[0.003436, 0.000077, 0.083892], axis=[0, -1, 0], range=[-0.78, 0.78])

    # env.add_actuator("motor_joint", "joint_middle_0")

    # env.exclude_contact("carpals", "middle_0")

    env.export_xml("xml/model.xml")

    # print(env.model_str)
    env.run_interactive()
    # env.run_simulation()