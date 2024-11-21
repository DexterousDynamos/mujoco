from src.Mujoco_XML import Mujoco_XML
from src.Fusion_Model import Fusion_Model
import numpy as np
from pyquaternion import Quaternion
import os
from typing import Dict, List, Tuple
from dataclasses import dataclass, field
import re
import datetime
from src.utils import reduce_mesh, find_latest_folder, bytes_to_mb
from termcolor import colored

@dataclass
class DexterousDynamos:
    # Neccesary inputs
    # /

    # Optional inputs
    model_name:             str     = "DexterousDynamos"

    output_dir:             str     = "output/"
    output_name:            str     = None
    asset_folder:           str     = "assets/"
    json_filename:          str     = "fusion_info.json"

    use_rel_stlpath:        bool    = False    # Use relative paths for the STL files in the XML file
    reduce_stls:            bool    = False    # Reduce the size of the STL files. If files are too big, mujoco will not result in an error
    max_stl_size:           int     = 5e6       # Bytes

    # Public variables, not to be set by user
    # /

    # Internal variables
    _env: Mujoco_XML = field(init=False)
    _fusion_data: Fusion_Model = field(init=False)

    def __post_init__(self):
        # Initialize the Mujoco XML environment
        self._env = Mujoco_XML(model_name=self.model_name)

        # Add assets to the Mujoco XML environment
        latest_folder = find_latest_folder(self.asset_folder)
        self.asset_folder = os.path.relpath(latest_folder) # os.path.abspath(latest_folder)
        self._add_assets()
        if self.reduce_stls:
            self._reduce_stls()

        # Read the Fusion JSON file
        self._fusion_data = Fusion_Model(json_file_path=os.path.join(self.asset_folder, self.json_filename))

        # Add components to the Mujoco XML environment
        self._recursive_add_component(self._fusion_data.joint_components[0])

    def _reduce_stls(self):
        '''
        Reduce the size of the STL files in the asset folder.
        '''
        n_reduced_stls = 0
        for root, _, files in os.walk(self.asset_folder):
            for file in files:
                if file.lower().endswith(".stl"):
                    full_filepath = os.path.abspath(os.path.join(root, file))
                    if os.path.getsize(full_filepath) > self.max_stl_size:
                        filesize = os.path.getsize(full_filepath)
                        factor = self.max_stl_size / filesize
                        reduce_mesh(full_filepath, full_filepath, factor, verbose=False)
                        print(colored("WARNING", "yellow") + f": File {full_filepath} was reduced from {bytes_to_mb(filesize):.2f} MB to {bytes_to_mb(os.path.getsize(full_filepath)):.2f} MB")
                        n_reduced_stls += 1
        if n_reduced_stls == 1:
            print("Reduced 1 STL file in the latest asset folder.")
        elif n_reduced_stls > 1:
            print(f"Reduced {n_reduced_stls} STL files in the latest asset folder.")
        else:
            print(colored("No STL files were reduced in size.", "cyan"))

    def _add_assets(self):
        '''
        Add assets to the Mujoco XML environment.
        '''
        for root, _, files in os.walk(self.asset_folder):
            for file in files:
                if file.lower().endswith(".stl"):
                    full_filepath = os.path.join(root, file)
                    if self.use_rel_stlpath:
                        full_filepath = os.path.relpath(full_filepath)
                    else:
                        full_filepath = os.path.abspath(full_filepath)
                    base_name = os.path.splitext(file)[0]
                    self._env.add_asset(base_name, full_filepath)

    def _recursive_add_component(self, component: Fusion_Model.Component) -> None:
        '''
        Adds a component and its children to the Mujoco XML environment.

        Args:
            component_name (str): The name of the component to add. Its children will also be added.
        '''
        quat, trans = component.relative_transform
        # parent_name = component.parent.id if component.parent is not None else ''
        parent_name = component.parent.name if component.parent is not None else 'root' # New
        # TODO: Add better 
        self._env.add_body(component.name, component.stlname, trans, quat, parent_name, exclude_contact=True)
        if component.joint is not None:
            self._env.add_joint(body_name=component.name, joint_name=component.joint.joint_name, pos=component.joint.relative_transform[1], axis=component.joint.relative_transform[0], range=component.joint.range)
            
            # Joint equalites hardcoded here for now, but will be implemented soon (hopefully)
            if "M-DP" in component.name:
                self._env.add_joint_equality(component.joint.joint_name, component.parent.joint.joint_name, 70/120)
            else:
                self._env.add_actuator(component.joint.joint_name + "_actuator", component.joint.joint_name, ctrlrange=component.joint.range)

        for child in component.children:
            self._recursive_add_component(child)

    def copy_assets(self, asset_folder: str = None, output_folder: str = None):
        '''
        Copy the assets from the specified folder to the asset folder of the model.

        Args:
            asset_folder (str): The path to the folder containing the assets. Defaults to the latest asset folder.
            output_folder (str): The path to the folder where the assets should be copied to. Defaults to the output folder of the model.
        '''
        if asset_folder is None:
            asset_folder = self.asset_folder
        if output_folder is None:
            output_folder = os.path.join(self.output_dir, self.asset_folder)

        if not os.path.exists(output_folder):
            os.makedirs(output_folder)
            
        for root, _, files in os.walk(asset_folder):
            for file in files:
                if file.lower().endswith(".stl"):
                    full_filepath = os.path.abspath(os.path.join(root, file))
                    output_filepath = os.path.join(output_folder, file)
                    os.system(f"cp {full_filepath} {output_filepath}")

    def export_xml(self, filename: str = None):
        '''
        Export the Mujoco XML environment to a file.

        Args:
            filename (str): The name of the file to export to.
        '''
        def check_dir(dir: str) -> str:
            if not dir.endswith("/"):
                dir += "/"
            if not os.path.exists(dir):
                os.makedirs(dir)

            return dir

        def check_filename(filename: str) -> str:
            if len(filename.split("/")) > 1:
                raise ValueError("The filename should not contain any directories. Use the 'output_dir' parameter instead.")
            extension_added = False
            if not filename.endswith(".xml"):
                filename += ".xml"
                extension_added = True

            elif len(filename.split(".")) > 2:
                error_str = "The 'filename' should not contain more than one '.'"
                if extension_added:
                    error_str += " - extension '.xml' was added automatically, as it was missing"
                raise ValueError(error_str)

            return filename

        output_dir = check_dir(self.output_dir)
        if filename is None:
            output_name = check_filename(self.model_name)
        else:
            output_name = check_filename(filename)

        filename = output_dir + output_name
        self._env.export_xml(filename)

    def run_interactive(self):
        '''
        Run the Mujoco simulation interactively.
        '''
        self._env.run_interactive()

if __name__ == "__main__":
    model = DexterousDynamos(reduce_stls=True, use_rel_stlpath=True)
    model.copy_assets()
    model.export_xml()
    model.run_interactive()
