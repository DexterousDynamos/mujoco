import json
import numpy as np
from dataclasses import dataclass, field
from typing import List, Dict
from pyquaternion import Quaternion

@dataclass
class Fusion_Model:
    # Neccesary inputs
    json_file_path: str = field()

    # Public variables, not to be set by user
    root_component: 'Fusion_Model.Component'        = field(init=False)
    components:     List['Fusion_Model.Component']  = field(init=False, default_factory=list)

    # root_joint_component: 'Fusion_Model.Component'          = field(init=False)
    # joint_components:     List['Fusion_Model.Component']    = field(init=False, default_factory=list)

    # Internal variables
    _json_data:         Dict            = field(init=False, default_factory=dict)
    _subassembly_names: List[str]       = field(init=False, default_factory=list)
    _component_indices:         Dict[str, int]  = field(init=False, default_factory=dict) # TODO: Make useless via unique ids in json file
    # _joint_component_indices:   Dict[str, int]  = field(init=False, default_factory=dict) # TODO: Make useless via unique ids in json file
    _parent_dict:       Dict[str, str]  = field(init=False, default_factory=dict) # TODO: Make useless via fixing weird errors in json file

    def __post_init__(self):
        # Read json data
        with open(self.json_file_path, 'r') as json_file:
            self._json_data = json.load(json_file)
        self._build_component_tree()
        # self._build_joint_tree()
        # Hardcode parent dict (for joints) for now
        self._parent_dict = {"01": "Root",
                            "02": "Root", "03": "Root", "04": "Root", "05": "Root", "06": "Root",
                            "07": "02", "08": "07", "09": "08", "10": "09",
                            "11": "03", "12": "11", "13": "12", "14": "13",
                            "15": "04", "16": "15", "17": "16", "18": "17",
                            "19": "05", "20": "19", "21": "20", "22": "21",
                            "23": "06", "24": "23", "25": "24", "26": "25"}
        # self._parent_dict = {"Root/Carpals:1": "Root",
        #                      "Root/M-Assembly:1": "Root", "Root/M-Assembly:2": "Root", "Root/I-Assembly:1": "Root", "Root/T-Assembly:1": "Root", "Root/P-Assembly:1": "Root",
        #                      "M-Assembly:1/M-DP:1": "M-Assembly:1/M-MP:1", "M-Assembly:1/M-MP:1": "M-Assembly:1/M-PP:1", "M-Assembly:1/M-PP:1": "M-Assembly:1/M-AP:1", "M-Assembly:1/M-AP:1": "Root/M-Assembly:1",
        #                      "M-Assembly:2/M-DP:1": "M-Assembly:2/M-MP:1", "M-Assembly:2/M-MP:1": "M-Assembly:2/M-PP:1", "M-Assembly:2/M-PP:1": "M-Assembly:2/M-AP:1", "M-Assembly:2/M-AP:1": "Root/M-Assembly:2",
        #                      "I-Assembly:1/M-DP:1": "I-Assembly:1/M-MP:1", "I-Assembly:1/M-MP:1": "I-Assembly:1/I-PP:1", "I-Assembly:1/I-PP:1": "I-Assembly:1/M-AP:1", "I-Assembly:1/M-AP:1": "Root/I-Assembly:1",
        #                      "T-Assembly:1/T-DP:1": "T-Assembly:1/T-PP:1", "T-Assembly:1/T-PP:1": "T-Assembly:1/T-AP_OUT:1", "T-Assembly:1/T-AP_OUT:1": "T-Assembly:1/T-AP:1", "T-Assembly:1/T-AP:1": "Root/T-Assembly:1",
        #                      "P-Assembly:1/M-DP:1": "P-Assembly:1/M-MP:1", "P-Assembly:1/M-MP:1": "P-Assembly:1/P-PP:1", "P-Assembly:1/P-PP:1": "P-Assembly:1/M-AP:1", "P-Assembly:1/M-AP:1": "Root/P-Assembly:1"}

        # {"Carpals v54:1": "Root",
        #     "I-Assembly v9:1": "Root", "P-Assembly v82:1": "Root", "P-Assembly v82:2": "Root",
        #     "M-DP v55:1_2": "M_MP v73:1_2", "M_MP v73:1_2": "I_PP v12:1", "I_PP v12:1": "M-AP v67:1_2", "M-AP v67:1_2": "I-Assembly v9:1",
        #     "M-DP v55:1_1": "M_MP v73:1_1", "M_MP v73:1_1": "M_PP v60:1_1", "M_PP v60:1_1": "M-AP v67:1_1", "M-AP v67:1_1": "P-Assembly v82:2",
        #     "M-DP v55:1": "M_MP v73:1", "M_MP v73:1": "M_PP v60:1", "M_PP v60:1": "M-AP v67:1", "M-AP v67:1": "P-Assembly v82:1"}

    def __str__(self):
        self._print_component_tree(self.components[0], 0)
        return ""

    @dataclass
    class Component:
        # TODO: Make "name" useless. Instead, make ID primary
        name: str = field()
        id:                 int                             = 0 # TODO: Make useless via unique ids in json file
        stlname:            str                             = None
        quaternion:         Quaternion                      = field(default_factory=lambda: Quaternion(1, 0, 0, 0).normalised)
        translation:        np.ndarray                      = field(default_factory=lambda: np.zeros(3))
        parent:             'Fusion_Model.Component'        = None
        children:           List['Fusion_Model.Component']  = field(default_factory=list)
        joint:              'Fusion_Model.Joint'            = None
        # index:              int                             = None # TODO: Make useless via unique ids in json file
        # unique_name:        str                             = None # TODO: Make useless via unique ids in json file

    @dataclass
    class Joint:
        pos:    np.ndarray                  = field(default_factory=lambda: np.zeros(3))
        axis:   np.ndarray                  = field(default_factory=lambda: np.zeros(3))
        parent: 'Fusion_Model.Component'    = None

    # def find_component_index(self, name: str) -> int:
    #     '''
    #     Find the index of a component in the component list.

    #     Args:
    #         name (str): The name of the component to find.

    #     Returns:
    #         int: The index of the component in the component list.
    #     '''
    #     for i, component in enumerate(self.components):
    #         if component.name == name:
    #             return i
            
    #     return None

    # TODO: Make useless via unique ids in json file
    def find_parent_index(self, name: str) -> int:
        '''
        Find the index of a component in the component list.

        Args:
            name (str): The name of the component to find.

        Returns:
            int: The index of the component in the component list.
        '''
        for i, component in enumerate(self.components):
            if component.name == name:
                return i
            
        return None
    
    # def find_joint_parent_index(self, name: str) -> int:
    #     '''
    #     Find the index of a joint component in the joint component list.

    #     Args:
    #         name (str): The name of the joint component to find.

    #     Returns:
    #         int: The index of the joint component in the joint component list.
    #     '''
    #     for i, component in enumerate(self.joint_components):
    #         if component.name == name:
    #             return i
            
    #     return None

    def _build_component_tree(self):
        '''
        Build the component tree from the JSON data.
        '''
        self.root_component = Fusion_Model.Component("Root")
        self.components.append(self.root_component)
        self._component_indices["Root"] = 0

        # TODO - have a look at M-AP-None glitch
        data = self._json_data["Components"]

        # TODO: _component_counts does not work if sub-assemblies have multiple instances in higher sub-assemblies. Need proper hierarchy or unique ids in json file (i.e. via component ids)
        
        _component_counts = {}
        for index, item in enumerate(data):
            name = item["Component Name"]
            stlname = item["STL File"]
            # # if name not in _component_counts.keys():
            # #     _component_counts[name] = 0
            # # _component_counts[name] += 1
            # # name += f"_{_component_counts[name]}"
            # if name in _component_counts:
            #     _component_counts[name] += 1
            #     name += f"_{_component_counts[name]}"
            # else:
            #     _component_counts[name] = 0

            # self.components.append(Fusion_Model.Component(name))
            # self.components[-1].id = index + 1 # +1 because root component is at index 0
            self.components.append(Fusion_Model.Component(name, stlname=stlname)) # [CHANGED FOR ID]

            if not item["Is Base Component"]:
                self._subassembly_names.append(name)

            parent_name = item["Parent"]
            # Add component to names_ids
            # self._component_indices[f"{parent_name}/{name}"] = len(self._component_indices) + 1 # index + 1 # +1 because root component is at index 0
            # # self.components[-1].index = len(self.components) - 1
            # # self.components[-1].unique_name = f"{parent_name}/{name}"

            self._component_indices[name] = len(self._component_indices) # [CHANGED FOR ID]

        # for component_name in _component_counts.keys():
        #     _component_counts[component_name] = 0

        # print(self._component_indices)

        _component_counts = {}
        for item in data:
            name = item["Component Name"]
            # # # if name not in _component_counts.keys():
            # # #     _component_counts[name] = 0
            # # _component_counts[name] += 1
            # # name += f"_{_component_counts[name]}"
            # if name in _component_counts:
            #     _component_counts[name] += 1
            #     name += f"_{_component_counts[name]}"
            # else:
            #     _component_counts[name] = 0

            # print("name:", name)
            # print("parent:", item["Parent"])
            # for comp in self.components:
            #     print(comp.name)

            # component_index = self.find_component_index(name)
            parent_name = item["Parent"]
            # print(self._component_indices)

            # component_index = self._component_indices[f"{parent_name}/{name}"]
            component_index = self._component_indices[name] # [CHANGED FOR ID]

            # if parent_name != "Root":
            #     print(_component_counts)
            #     parent_name += f"_{_component_counts[parent_name]}"
            # parent_index = self.find_component_index(parent_name)

            # parent_index = self.find_parent_index(parent_name)
            parent_index = self._component_indices[parent_name] # [CHANGED FOR ID]

            # print("Parent index for component", parent_name, ":", parent_index)
            # print("Check:", self.components[parent_index].name)

            # Add parent to component
            # print("parent name:", parent_name)
            # print("parent index:", parent_index)
            # print("component index:", component_index)
            self.components[component_index].parent = self.components[parent_index]
            # if parent_name == "/":
            #     # print(self.components[self.components.index(Fusion_Model.Component(name))])
            #     self.components[component_index].parent = self.components[parent_index]
            # else:
            #     self.components[component_index].parent = self.components[parent_index]

            # Add child to parent
            self.components[parent_index].children.append(self.components[component_index])
            
            # Convert transformations into numpy arrays
            transformation = {
                "Quaternion":       np.array(item["Transformation"]["Quaternion"]),
                "Translation":      np.array(item["Transformation"]["Translation"])
            }
            self.components[component_index].quaternion         = Quaternion(transformation["Quaternion"]).normalised
            # self.components[component_index].translation        = np.array(transformation["Translation"]) / 100 # Convert from dm to mm
            self.components[component_index].translation        = np.array(transformation["Translation"]) # / 100 # Convert from dm to mm # [CHANGED FOR ID]

        # print(self._component_indices)

    def remap_component_tree(self):
        '''
        Remap the component tree to give each component a unique parent, based on the parent_dict {"Child": "Parent"}.

        Args:
            parent_dict (Dict[str, str]): A dictionary of the form {"Child": "Parent"}.
        '''
        # for comp in self.components:
        #     if comp.name == "Root":
        #         continue
        #     print(f"Parent of {comp.unique_name}: {comp.parent.name} ({comp.parent.unique_name})")
        
        for comp in self.components:
            if comp.name == "Root":
                continue
            # if self.find_component_index(parent_dict[comp.name]) == -1:
            #     raise ValueError(f"DEBUG: Parent component '{parent_dict[comp.name]}' not found in component list")
            if comp.parent.name == "Root":
                parent_index = 0
            else:
                # comp_name = f'{comp.parent.name}/{comp.name}'
                comp_name = comp.name # [CHANGED FOR ID]
                parent_index = self._component_indices[self._parent_dict[comp_name]]
            comp.parent = self.components[parent_index]
            # comp.unique_name = f'{comp.parent.name}/{comp.name}'

        for comp in self.components:
            comp.children = []
        for comp_name, parent_name in self._parent_dict.items():
            if parent_name == "Root":
                parent_index = 0
            else:
                parent_index = self._component_indices[parent_name]
            comp_index = self._component_indices[comp_name]

            self.components[parent_index].children.append(self.components[comp_index])

    # def remap_component_tree(self, parent_dict: Dict[str, str]):
    #     '''
    #     Remap the component tree to give each component a unique parent, based on the parent_dict {"Child": "Parent"}.

    #     Args:
    #         parent_dict (Dict[str, str]): A dictionary of the form {"Child": "Parent"}.
    #     '''
    #     for comp in self.components:
    #         if comp.name == "Root":
    #             continue
    #         if self.find_component_index(parent_dict[comp.name]) == -1:
    #             raise ValueError(f"DEBUG: Parent component '{parent_dict[comp.name]}' not found in component list")
    #         comp.parent = self.components[self.find_component_index(parent_dict[comp.name])]

    #     for comp_name in parent_dict.values():
    #         self.components[self.find_component_index(comp_name)].children = [comp for comp in self.components if comp.parent == self.components[self.find_component_index(comp_name)]]

    #     for comp_name in parent_dict.values():
    #         self.components[self.find_component_index(comp_name)].children = [comp for comp in self.components if comp.parent == self.components[self.find_component_index(comp_name)]]

    def _build_joint_tree(self):
        '''
        Build the joint tree from the JSON data.
        '''
        data = self._json_data["Joints"]

        # # Add joints to components
        # for item in data:
        #     comp_name = f'{item["Rotating Component"]["Parent"]}/{item["Rotating Component"]["Component Name"]}'
        #     self.components[self._component_indices[comp_name]].joint = Fusion_Model.Joint(pos=np.array(item["Transformation"]["Joint Origin"]), axis=np.array(item["Transformation"]["Joint Axis"]))

        # Get joint hierarchy
        self.root_joint_component = Fusion_Model.Component("Root")
        self.joint_components.append(self.root_joint_component)

        # Add subassembly names as components, as they are not included in the joint data
        for subassembly_name in self._subassembly_names:
            self.joint_components.append(Fusion_Model.Component(subassembly_name))
            self._joint_component_indices[f"Root/{subassembly_name}"] = len(self.joint_components) - 1

        # print(f"Subassembly names: {self._subassembly_names}")
        # print("Length of joint components:", len(self.joint_components))

        # TODO: Fix to add carpals
        # for index, item in enumerate(data, len(self._subassembly_names) + 1): # +1 because root component is at index 0
        for index, item in enumerate(data, len(self.joint_components)):
            name = item["Rotating Component"]["Component Name"]
            parent_name = item["Rotating Component"]["Parent"]

            self._joint_component_indices[f"{parent_name}/{name}"] = index
            self.joint_components.append(Fusion_Model.Component(name))

        # Add Base components if not already in joint data
        for index, item in enumerate(data, len(self.joint_components)):
            name = item["Base Component"]["Component Name"]
            parent_name = item["Base Component"]["Parent"]

            try:
                self._joint_component_indices[f"{parent_name}/{name}"]
            except KeyError:
                self._joint_component_indices[f"{parent_name}/{name}"] = index
                self.joint_components.append(Fusion_Model.Component(name))

        for index_key, index_value in self._joint_component_indices.items():
            print(f"Index for {index_key}: {index_value}")
            print(f"Check: {self.joint_components[index_value].name}")

        # for item in data:
        #     name = item["Rotating Component"]["Component Name"]
        #     parent_name = item["Rotating Component"]["Parent"]
        #     component_index = self._joint_component_indices[f"{parent_name}/{name}"]
        #     # print(self._joint_component_indices)
        #     # for comp in self.joint_components:
        #     #     print(comp.name)
        #     parent_index = self.find_joint_parent_index(parent_name)
        #     # print("Parent index for joint component", parent_name, ":", parent_index)
        #     self.joint_components[component_index].parent = self.joint_components[parent_index]
        #     self.joint_components[parent_index].children.append(self.joint_components[component_index])
            
        #     self.joint_components[self._joint_component_indices[f"{parent_name}/{name}"]].joint = Fusion_Model.Joint(pos=np.array(item["Transformation"]["Joint Origin"]), axis=np.array(item["Transformation"]["Joint Axis"]))

        contained_joints = {}
        overlapping_joints = {}
        for item in data:
            component_name1 = item["Base Component"]["Component Name"]
            parent_name1 = item["Base Component"]["Parent"]
            component_name2 = item["Rotating Component"]["Component Name"]
            parent_name2 = item["Rotating Component"]["Parent"]
            if parent_name1 != parent_name2:
                overlapping_joints[f"{parent_name1}/{parent_name2}"] = [component_name1, component_name2] # item
            else:
                try:
                    # contained_joints[parent_name1].append([component_name1, component_name2, item["Transformation"]])
                    contained_joints[parent_name1].append([component_name1, component_name2])
                except KeyError:
                    # contained_joints[parent_name1] = [[component_name1, component_name2, item["Transformation"]]]
                    contained_joints[parent_name1] = [[component_name1, component_name2]]
                # contained_joints[component_name1 + "/" + component_name2] = item["Transformation"]

        def extract_hierarchy(pairs):
            # Dictionary to store each node's child (e.g., {"n1": "n2"})
            parent_to_child = {}
            # Set to track all nodes that have parents (i.e., are not root)
            children = set()

            # Populate the dictionary and the children set
            for parent, child in pairs:
                parent_to_child[parent] = child
                children.add(child)

            # Find the root (a node that is a parent but never a child)
            root = None
            for parent in parent_to_child.keys():
                if parent not in children:
                    root = parent
                    break

            # Construct the hierarchy starting from the root
            hierarchy = []
            current = root
            while current:
                hierarchy.append(current)
                current = parent_to_child.get(current)  # Move to the next child in the chain

            return hierarchy

        # TODO: DEBUG following
        for parent_name, component_names in contained_joints.items():
            hierarchy = extract_hierarchy(component_names)
            for index, comp_name in enumerate(hierarchy):
                comp_name = f"{parent_name}/{comp_name}"
                if index != 0:
                    self.joint_components[self._joint_component_indices[comp_name]].parent = self.joint_components[self._joint_component_indices[f"{parent_name}/{hierarchy[index - 1]}"]]
                if index != len(hierarchy) - 1:
                    # self._joint_component_indices[comp_name]
                    # self.joint_components[self._joint_component_indices[comp_name]]
                    # hierarchy[index + 1]
                    # self._joint_component_indices[f"{parent_name}/{hierarchy[index + 1]}"]
                    # self.joint_components[self._joint_component_indices[f"{parent_name}/{hierarchy[index + 1]}"]]
                    self.joint_components[self._joint_component_indices[comp_name]].children.append(self.joint_components[self._joint_component_indices[f"{parent_name}/{hierarchy[index + 1]}"]])
        
        print(self._joint_component_indices)

        for parent_names, joint_names in overlapping_joints.items():
            parent_name1, parent_name2 = parent_names.split("/")
            self.joint_components[self._joint_component_indices[f"{parent_name1}/{joint_names[0]}"]].parent = self.joint_components[self._joint_component_indices[f"{parent_name2}/{joint_names[1]}"]]
            self.joint_components[self._joint_component_indices[f"{parent_name2}/{joint_names[1]}"]].children.append(self.joint_components[self._joint_component_indices[f"{parent_name1}/{joint_names[0]}"]])
        # for subassembly, joint_data in contained_joints.items():
        #     for joint in joint_data:
        #         pass

        # Make dict where key is the parent name and value is dict with pos and axis
        pass

    def _print_component_tree(self, component: 'Fusion_Model.Component', level: int):
        '''
        Print the component tree in a human-readable format.

        Args:
            component (Fusion_Model.Component): The component to print.
            level (int):                        The level of the component in the tree.
        '''
        if component is None:
            return
        # print("  " * level + '- ' + component.name)
        stlname = component.stlname.split(".stl")[0] if component.stlname is not None else None # [CHANGED FOR ID]
        print(f"{'  ' * level}- {component.name} ({stlname})") # [CHANGED FOR ID]
        for child in component.children:
            # if child.name == "M-MP:1":
            #     print("CHILDREN OF M-MP:1:", len(child.children))
            # print(child.name)
            self._print_component_tree(child, level + 1)

    def print_detailed_info(self):
        '''
        Print detailed information about the components in the Fusion Model.
        '''
        print("\nComponent INFO")
        for component in self.components:
            print()
            print("Component.name:\t\t", component.name)
            print("Component.parent:\t", component.parent.name if component.parent is not None else None)
            print("Component.children:\t", [component.children[i].name for i in range(len(component.children))])
            print("Component.translation:\t", np.around(component.translation, 3))
            print("Component.quaternion:\t", np.around(component.quaternion.elements, 3))
            # print("Component.joint.pos:\t", component.joint.position if component.joint is not None else None)
            # print("Component.joint.axis:\t", component.joint.axis if component.joint is not None else None)

        # print("\n\nJoint INFO")
        # for component in self.joint_components:
        #     print()
        #     print("Rotating component:\t", component.name)
        #     print("Base component:\t\t", component.parent.name if component.parent is not None else None)
        #     print("Joint.children:\t\t", [component.children[i].name for i in range(len(component.children))])
        #     if component.joint is not None:
        #         print("Joint.pos:\t", np.around(component.joint.pos, 3))
        #         print("Joint.axis:\t", np.around(component.joint.axis, 3))

if __name__ == "__main__":
    # fusion_model = Fusion_Model(json_file_path='assets/fusion_export_2024-11-13_13-04-40/fusion_info.json')
    fusion_model = Fusion_Model(json_file_path='assets/fusion_export_2024-11-14_21-01-59/fusion_info.json')
    print(fusion_model)
    # fusion_model.print_detailed_info()

    # fusion_model2 = Fusion_Model(json_file_path='assets/fusion_export_2024-11-13_13-04-40/fusion_info.json')
    fusion_model2 = Fusion_Model(json_file_path='assets/fusion_export_2024-11-14_21-01-59/fusion_info.json')
    fusion_model2.remap_component_tree()
    print(fusion_model2)