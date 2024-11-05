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
    # root_component: 'Fusion_Model.Component' = field(init=False, default=None)
    root_component: 'Fusion_Model.Component'        = field(init=False)
    components:     List['Fusion_Model.Component']  = field(init=False, default_factory=list)

    # Internal variables
    _component_counts: Dict[str, int] = field(init=False, default_factory=dict)

    def __post_init__(self):
        self.root_component = Fusion_Model.Component("Root")
        self.components.append(self.root_component)
        self._build_component_tree()

    def __str__(self):
        self._print_component_tree(self.components[0], 0)
        return ""

    @dataclass
    class Component:
        name: str = field()
        rotation_matrix:    np.ndarray                      = field(default_factory=lambda: np.eye(3))
        quaternion:         Quaternion                      = field(default_factory=lambda: Quaternion(1, 0, 0, 0).normalised)
        translation:        np.ndarray                      = field(default_factory=lambda: np.zeros(3))
        parent:             'Fusion_Model.Component'        = None
        children:           List['Fusion_Model.Component']  = field(default_factory=list)

    def find_component_index(self, name: str) -> int:
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

    def _build_component_tree(self):
        '''
        Build the component tree from the JSON data. TODO: Line needs fix if more assembly layers
        '''
        # Load the JSON data from the specified file
        with open(self.json_file_path, 'r') as json_file:
            data = json.load(json_file)
        
        for item in data:
            name = item["Component Name"]
            if name in self._component_counts:
                self._component_counts[name] += 1
                name += f"_{self._component_counts[name]}"
            else:
                self._component_counts[name] = 0
            self.components.append(Fusion_Model.Component(name))

        self._component_counts = {}
        for item in data:
            name = item["Component Name"]
            if name in self._component_counts:
                self._component_counts[name] += 1
                name += f"_{self._component_counts[name]}"
            else:
                self._component_counts[name] = 0
            component_index = self.find_component_index(name)
            parent_name = item["Parent(s)"] # TODO: Needs fix if more assembly layers
            parent_index = self.find_component_index(parent_name)

            # Add parent to component
            if parent_name == "/":
                # print(self.components[self.components.index(Fusion_Model.Component(name))])
                self.components[component_index].parent = self.components[parent_index]
            else:
                self.components[component_index].parent = self.components[parent_index]

            # Add child to parent
            self.components[parent_index].children.append(self.components[component_index])
            
            # Convert transformations into numpy arrays
            transformation = {
                "Rotation Matrix":  np.array(item["Transformation"]["Rotation Matrix"]),
                "Quaternion":       np.array(item["Transformation"]["Quaternion"]),
                "Translation":      np.array(item["Transformation"]["Translation"])
            }
            self.components[component_index].rotation_matrix    = np.array(transformation["Rotation Matrix"])
            self.components[component_index].quaternion         = Quaternion(transformation["Quaternion"]).normalised
            self.components[component_index].translation        = np.array(transformation["Translation"]) / 100 # Convert from dm to mm

    def remap_component_tree(self, parent_dict: Dict[str, str]):
        '''
        Remap the component tree to give each component a unique parent, based on the parent_dict {"Child": "Parent"}.

        Args:
            parent_dict (Dict[str, str]): A dictionary of the form {"Child": "Parent"}.
        '''
        for comp in self.components:
            if comp.name == "Root":
                continue
            if self.find_component_index(parent_dict[comp.name]) == -1:
                raise ValueError(f"DEBUG: Parent component '{parent_dict[comp.name]}' not found in component list")
            comp.parent = self.components[self.find_component_index(parent_dict[comp.name])]

        for comp_name in parent_dict.values():
            self.components[self.find_component_index(comp_name)].children = [comp for comp in self.components if comp.parent == self.components[self.find_component_index(comp_name)]]
        
    # def copy(self):
    #     return Fusion_Model(json_file_path=self.json_file_path)

    def _print_component_tree(self, component: 'Fusion_Model.Component', level: int):
        '''
        Print the component tree in a human-readable format.

        Args:
            component (Fusion_Model.Component): The component to print.
            level (int):                        The level of the component in the tree.
        '''
        if component is None:
            return
        print("  " * level + '- ' + component.name)
        for child in component.children:
            # print(child.name)
            self._print_component_tree(child, level + 1)

    def print_detailed_info(self):
        '''
        Print detailed information about the components in the Fusion Model.
        '''
        for component in self.components:
            print()
            print("Component.name:\t\t", component.name)
            print("Component.parent:\t", component.parent.name if component.parent is not None else None)
            print("Component.children:\t", [component.children[i].name for i in range(len(component.children))])
            print("Component.translation:\t", np.around(component.translation, 3))
            print("Component.quaternion:\t", np.around(component.quaternion.elements, 3))
            print("Component.rotation_matrix:\n", np.around(component.rotation_matrix, 3))

if __name__ == "__main__":
    fusion_model = Fusion_Model(json_file_path='assets/fusion_info.json')
    print(fusion_model)
    fusion_model.print_detailed_info()