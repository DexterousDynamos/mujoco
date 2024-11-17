import json
import numpy as np
from dataclasses import dataclass, field
from typing import List, Dict, Tuple
from pyquaternion import Quaternion
from termcolor import colored

@dataclass
class Fusion_Model:
    # Neccesary inputs
    json_file_path: str = field()

    # Public variables, not to be set by user
    components:             List['Fusion_Model.Component']  = field(init=False, default_factory=list) # Root component is always first
    joint_components:       List['Fusion_Model.Component']  = field(init=False, default_factory=list) # Root joint component is always first

    # Internal variables
    _json_data:                 Dict            = field(init=False, default_factory=dict)
    _component_indices:         Dict[str, int]  = field(init=False, default_factory=dict) # Not necessary, but useful and faster than iterating through list
    _joint_component_indices:   Dict[str, int]  = field(init=False, default_factory=dict) # Not necessary, but useful and faster than iterating through list

    def __post_init__(self):
        # Read json data
        with open(self.json_file_path, 'r') as json_file:
            self._json_data = json.load(json_file)
        self._build_component_tree()
        self._build_joint_tree()
        self._calculate_transforms()

    def __str__(self):
        print(colored("\nComponent Tree", "blue"))
        self._print_component_tree(self.components[0], 0)
        print(colored("\nJoint Tree", "blue"))
        self._print_joint_tree(self.joint_components[0], 0)
        return ""

    @dataclass
    class Component:
        id:                 int                             = field()
        stlname:            str                             = field(default=None)
        transform:          Tuple[Quaternion, np.ndarray]   = field(init=False, default_factory=lambda: (Quaternion(1, 0, 0, 0).normalised, np.zeros(3))) # (quaternion, translation)
        absolute_transform: Tuple[Quaternion, np.ndarray]   = field(init=False, default_factory=lambda: (Quaternion(1, 0, 0, 0).normalised, np.zeros(3))) # (quaternion, translation)
        relative_transform: Tuple[Quaternion, np.ndarray]   = field(init=False, default_factory=lambda: (Quaternion(1, 0, 0, 0).normalised, np.zeros(3))) # (quaternion, translation)
        parent:             'Fusion_Model.Component'        = field(init=False, default=None)
        children:           List['Fusion_Model.Component']  = field(init=False, default_factory=list)
        joint:              'Fusion_Model.Joint'            = field(init=False, default=None)

    @dataclass
    class Joint:
        joint_name:         str                             = field()
        transform:          Tuple[np.ndarray, np.ndarray]   = field(default_factory=lambda: (np.zeros(3), np.zeros(3))) # (axis, pos)
        relative_transform: Tuple[np.ndarray, np.ndarray]   = field(default_factory=lambda: (np.zeros(3), np.zeros(3))) # (axis, pos)
        range:               List[float]                    = field(default_factory=lambda: [-np.pi, np.pi])

    def _build_component_tree(self):
        '''
        Build the component tree from the JSON data.
        '''
        data = self._json_data["Components"]

        # Add components to tree
        self.root_component = Fusion_Model.Component("Root", None)
        self.components.append(self.root_component)
        self._component_indices["Root"] = 0
        for item in data:
            id = item["Component Name"]
            stlname = item["STL File"]
            self.components.append(Fusion_Model.Component(id, stlname))
            self._component_indices[id] = len(self._component_indices)

        # Remap parent to child
        for item in data:
            component_index = self._component_indices[item["Component Name"]]
            parent_index = self._component_indices[item["Parent"]]

            # Add parent to child
            self.components[component_index].parent = self.components[parent_index]

            # Add child to parent
            self.components[parent_index].children.append(self.components[component_index])

            # Add transformation
            quat = Quaternion(item["Transformation"]["Quaternion"]).normalised
            trans = np.array(item["Transformation"]["Translation"]) / 100 # Convert from dm to mm
            self.components[component_index].transform = (quat, trans)

    def _build_joint_tree(self):
        '''
        Build the joint tree from the JSON data.
        '''
        data = self._json_data["Joints"]

        # Add joints components to tree with transformation
        for item in data:
            base_component_id = item["Base Component"]
            rotating_component_id = item["Rotating Component"]

            if rotating_component_id not in self._joint_component_indices:
                stlname = self.components[self._component_indices[rotating_component_id]].stlname.split(".stl")[0]
                self.joint_components.append(Fusion_Model.Component(rotating_component_id, stlname))
                self.joint_components[-1].parent = base_component_id # 'str' for now, later becomes 'Fusion_Model.Component'

                axis = np.array(item["Transformation"]["Joint Axis"])
                pos = np.array(item["Transformation"]["Joint Origin"]) / 100 # Convert from dm to mm
                range = item["Transformation"]["Joint Range"]
                self.joint_components[-1].joint = Fusion_Model.Joint(rotating_component_id + "_joint", (axis, pos), range)

                self._joint_component_indices[rotating_component_id] = len(self._joint_component_indices) + 1

        # Add base
        root_added = False
        for item in data:
            base_component_id = item["Base Component"]

            if base_component_id not in self._joint_component_indices:
                if not root_added:
                    stlname = self.components[self._component_indices[base_component_id]].stlname.split(".stl")[0]
                    self.joint_components.insert(0, Fusion_Model.Component(base_component_id, stlname))
                    self._joint_component_indices[base_component_id] = 0
                    root_added = True
                else:
                    raise ValueError("Multiple root components found in joint data. This sort of model is not supported - all joints must lead to a single root component.")

        # Remap parent to child
        for joint_component in self.joint_components:
            if joint_component.parent is not None:
                # Add parent to child
                joint_component.parent = self.joint_components[self._joint_component_indices[joint_component.parent]]

                # Add child to parent
                joint_component.parent.children.append(joint_component)

    def _calculate_transforms(self):
        '''
        Calculate the relative (and absolute) transforms of all components and joints in the Fusion Model.
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
            if component.id == "Root":
                return component.transform[0]
            return recursive_calculate_quat(component.parent) * component.transform[0]
        
        def recursive_calculate_trans(component: Fusion_Model.Component, component_translation: np.ndarray) -> np.ndarray:
            '''
            Recursively calculates the total translation of the component.

            Args:
                component (Fusion_Model.Component): The component to calculate the translation of.
                component_translation (np.ndarray): The translation of the component.

            Returns:
                np.ndarray: The total translation of the component.
            '''
            if component.id == "Root":
                return component_translation
            return recursive_calculate_trans(component.parent, component.parent.transform[0].rotate(component_translation) + component.parent.transform[1])
        
        # Calculate absolute transforms
        for component in self.components:
            quat = recursive_calculate_quat(component)
            trans = recursive_calculate_trans(component, component.transform[1])

            joint_component = self.joint_components[self._joint_component_indices[component.id]] if component.id in self._joint_component_indices else None
            if joint_component is not None: # Not subassembly
                joint_component.absolute_transform = (quat, trans)

        # Calculate relative transforms
        for component in self.joint_components:
            if component.parent is None:
                continue

            relative_trans, relative_quat = get_relative_transform(component.parent.absolute_transform[1], component.parent.absolute_transform[0], component.absolute_transform[1], component.absolute_transform[0])
            component.relative_transform = (relative_quat, relative_trans)

        # Calculate joint transforms
        pass

    def detailed_name(self, component: 'Fusion_Model.Component', level: int = None) -> str:
        '''
        Print the detailed name of a component in the Fusion Model, i.e. the component ID and its shortened stl name.

        Args:
            component (Fusion_Model.Component): The component to print.

        Returns:
            str: The detailed name of the component.
        '''
        stlname = component.stlname.split(".stl")[0] if component.stlname is not None else "None"
        if level is None:
            return f"{component.id} ({stlname})"
        else:
            return f"{'  ' * level}{colored('-', 'cyan')} {component.id} {colored('(' + stlname + ')', 'green')}"

    def _print_component_tree(self, component: 'Fusion_Model.Component', level: int):
        '''
        Print the component tree in a human-readable format.

        Args:
            component (Fusion_Model.Component): The component to print.
            level (int):                        The level of the component in the tree.
        '''
        if component is None:
            return
        
        print(self.detailed_name(component, level))
        for child in component.children:
            self._print_component_tree(child, level + 1)

    def _print_joint_tree(self, component: 'Fusion_Model.Component', level: int):
        '''
        Print the joint tree in a human-readable format.

        Args:
            component (Fusion_Model.Component): The component to print.
            level (int):                        The level of the component in the tree.
        '''
        if component is None:
            return
        
        equivalent_component = self.components[self._component_indices[component.id]] if component.id in self._component_indices else None
        print(self.detailed_name(equivalent_component, level))
        for child in component.children:
            self._print_joint_tree(child, level + 1)

    def print_detailed_info(self):
        '''
        Print detailed information about the components in the Fusion Model.
        '''
        print(colored("\nComponent INFO", "green"))
        for component in self.components:
            print(colored("Component ID:\t\t", "cyan"), self.detailed_name(component))
            print(colored("Component Parent:\t", "cyan"), self.detailed_name(component.parent) if component.parent is not None else None)
            print(colored("Component Children:\t", "cyan"), [self.detailed_name(component.children[i]) for i in range(len(component.children))])
            print(colored("Component Translation:\t", "cyan"), np.around(component.transform[1], 3))
            print(colored("Component Quaternion:\t", "cyan"), np.around(component.transform[0].elements, 3))
            print()

        print(colored("\nJoint INFO", "green"))
        for component in self.joint_components:
            equivalent_component = self.components[self._component_indices[component.id]] if component.id in self._component_indices else None
            equivalent_parent = self.components[self._component_indices[component.parent.id]] if (component.parent is not None and component.parent.id in self._component_indices) else None
            print(colored("Joint Base:\t", "cyan"), self.detailed_name(equivalent_parent) if equivalent_parent is not None else None)
            print(colored("Joint Rotating:\t", "cyan"), self.detailed_name(equivalent_component))
            print(colored("Joint pos:\t", "cyan"), np.around(component.joint.transform[1], 3) if component.joint is not None else None)
            print(colored("Joint axis:\t", "cyan"), np.around(component.joint.transform[0], 3) if component.joint is not None else None)
            print(colored("Joint range:\t", "cyan"), np.around(component.joint.range, 3) if component.joint is not None else None)
            print()

if __name__ == "__main__":
    fusion_model = Fusion_Model(json_file_path='assets/fusion_export_2024-11-14_21-01-59/fusion_info.json')
    print(fusion_model)
    fusion_model.print_detailed_info()