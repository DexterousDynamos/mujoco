from Mujoco_XML import Mujoco_XML
from Fusion_Model import Fusion_Model
import numpy as np
# import json
import os
from typing import Dict

# Translation and quat to pass are global
# Translation within fusion?
# Rotation within fusion?

def quaternion_mult(q1, q2):
    # Extract individual components
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    
    # Perform the quaternion multiplication
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    
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

def add_component(env: Mujoco_XML, component: Fusion_Model.Component, lookup_table: Dict[str, str] = {}):
    if component.name == "link_0 v2:1" or component.name == "link_1 v2:1" or component.name == "link_2 v3:1":
        # print(f"\nTranslation for {component.name}: ", np.around(component.translation*1000, 3))
        env.add_body(lookup_table[component.name], pos=component.translation, quat=component.quaternion)
    elif component.name == "link_3_0 v3:1" or component.name == "link_3_1 v4:1":
        parent = component.parent
        print("\nParent translation: ", np.around(parent.translation*1000, 3))
        print("Parent quaternion: ", parent.quaternion)

        quat = quaternion_mult(parent.quaternion, component.quaternion) # component.quaternion
        print(f"Quaternion for {component.name}:", np.around(quat, 3))

        # trans = component.translation + apply_quaternion(parent.quaternion, parent.translation) + np.array([300, 0, 0])/1000
        trans = apply_quaternion(parent.quaternion, component.translation) + parent.translation
        # trans = apply_quaternion(component.quaternion, component.translation) + apply_quaternion(parent.quaternion, parent.translation) # + np.array([300, 0, 0])/1000
        print(f"Translation for {component.name}: ", np.around(trans*1000, 3))

        env.add_body(lookup_table[component.name], pos=trans, quat=quat)
    for child in component.children:
        add_component(env, child, lookup_table)

if __name__ == "__main__":
    ############################################ Read out json file ############################################
    data = Fusion_Model(json_file_path='assets/fusion_info_test.json')
    print(data)
    # exit()
    # print(data.root_component.name)
    # print([data.root_component.children[i].name for i in range(len(data.root_component.children))])

    ############################################ Create Mujoco XML ############################################
    env = Mujoco_XML(model_name='main.xml')
    # env.add_default_class("index_joint")
    # env.add_default("joint", "index_joint", axis="1 0 0")

    for root, _, files in os.walk("assets/Test/"):
        for file in files:
            if file.lower().endswith(".stl"):
                # full_filepath = os.path.join(root, file)
                full_filepath = os.path.abspath(os.path.join(root, file))
                base_name = os.path.splitext(file)[0].lower()
                env.add_asset(base_name, full_filepath)
    # env.add_asset("carpals", "assets/Carpals.stl")
    # env.add_asset("middle_0", "assets/Middle/Middle_0.stl")
    # lookup_table = {"Carpals v54:1": "carpals",
    #     "M-AP v67:1_2": "index_0", "I_PP v12:1": "index_1", "M_MP v73:1_2": "index_2", "M-DP v55:1": "index_3",
    #     "M-AP v67:1": "middle_0", "M_PP v60:1": "middle_1", "M_MP v73:1": "middle_2", "M-DP v55:1": "middle_3",
    #     "M-AP v67:1_1": "ring_0", "M_PP v60:1_1": "ring_1", "M_MP v73:1_1": "ring_2", "M-DP v55:1_1": "ring_3"}
    
    lookup_table = {"link_0 v2:1": "link_0", "link_1 v2:1": "link_1", "link_2 v3:1": "link_2", "link_3_0 v3:1": "link_3_0", "link_3_1 v4:1": "link_3_1"}
    
    base_quat = np.array([1, 0, 0, 1])*np.sqrt(2)/2 # [x, y, z, w], Fusion360 to Mujoco
    for child in data.root_component.children:
        add_component(env, child, lookup_table)

    # base_quat = np.array([0, 0, 0, 1])
    print(base_quat)
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