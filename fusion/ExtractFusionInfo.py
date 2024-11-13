import adsk.core, adsk.fusion, traceback
import re
import os
import json
from collections import defaultdict
from adsk.fusion import Occurrence
import datetime
import time

# Main function to run the script
def run(context):
    try:
        # Setup
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = app.activeProduct
        if not isinstance(design, adsk.fusion.Design):
            ui.messageBox('No active Fusion design', 'Error')
            return

        # Set the desired component names
        desired_component_names = [
            'M-MP', 'M-DP', 'M-AP','M-PP', 'I-PP', 'P-PP', 'T-PP', 'T-DP', 'T-AP', 'T-AP_OUT', 'Carpals', 'P-Assembly', 'I-Assembly', 'M-Assembly', 'T-Assembly'
        ]

        # Set the output directory
        repo_folder = ""
        with open(os.path.join(os.path.dirname(os.path.realpath(__file__)), "MUJOCO_REPO_PATH.txt"), "r") as f:
            repo_folder = f.read()
            repo_folder = repo_folder.strip()
        curr_time = datetime.datetime.now()
        formatted_time = curr_time.strftime("%Y-%m-%d_%H-%M-%S")
        folder_name = f"fusion_export_{formatted_time}"
        
        output_folder = os.path.join(repo_folder, "assets", folder_name)
        print(f"Output folder: {output_folder}")
        os.makedirs(output_folder, exist_ok=True)
        
        output_data = {
            "Components": [],
            "Joints": []
        }

        # Root component and all occurrences
        rootComp = design.rootComponent
        allOccs = rootComp.allOccurrences
        joints = rootComp.allJoints
        
        def clean_name(name):
            # Extract the number after the first colon and add it to the letters before the first empty space with a hyphen
            match = re.match(r'(\S+)\s.*:(\d+)', name)
            if match:
                return f"{match.group(1)}-{match.group(2)}"
            return name            

        # Collect component data
        def extract_component_data(occ: Occurrence):
            # Extract the base component name without the version info
            comp_name_with_version = occ.name  # Includes version (e.g., "M_MP v76:1")
            base_name = re.sub(r'\s*v\s*\d+:\d+', '', comp_name_with_version).strip()

            # Only proceed if the base name is in the desired component names list
            if base_name not in desired_component_names:
                return None

            # Extract the transformation data
            transform = occ.transform
            rotation_matrix = [
                [transform.getCell(0, 0), transform.getCell(0, 1), transform.getCell(0, 2)],
                [transform.getCell(1, 0), transform.getCell(1, 1), transform.getCell(1, 2)],
                [transform.getCell(2, 0), transform.getCell(2, 1), transform.getCell(2, 2)]
            ]
            quaternion = rotation_matrix_to_quaternion(rotation_matrix)
            translation = (transform.getCell(0, 3), transform.getCell(1, 3), transform.getCell(2, 3))

            plus_index = occ.fullPathName.rfind("+")
            if plus_index == -1:
                parent_name = 'Root'
                unique_name = occ.fullPathName
            else:
                parent_name = occ.fullPathName[:plus_index]
                unique_name = occ.fullPathName[plus_index+1:]         

            # Set up the STL export
            export_mgr = app.activeProduct.exportManager
            stl_export_options = export_mgr.createSTLExportOptions(occ)
            stl_export_options.isBinaryFormat = True  # Set binary STL format
            stl_filename = f"{base_name}.stl"  # Filename only
            stl_file_path = os.path.join(output_folder, stl_filename)  # Save in output directory
            stl_export_options.filename = stl_file_path
            export_mgr.execute(stl_export_options)

            entry = {
                "Component Name": clean_name(comp_name_with_version),
                "Parent(s)": clean_name(parent_name),
                "Transformation": {
                    # "Rotation Matrix": rotation_matrix,
                    "Quaternion": quaternion,
                    "Translation": translation
                },
                "STL File": stl_filename  # Add STL file path to the component data
            }
            return entry
        
        def extract_joint_data(joint):
            if joint.jointMotion.jointType == 1: # revolute joint
                # Find desired component names for both connected occurrences
                comp1 = joint.occurrenceOne.name
                comp2 = joint.occurrenceTwo.name
                
                try:
                    joint_origin = joint.geometryOrOriginOne.origin
                    joint_axis = joint.jointMotion.rotationAxisVector
                    joint_info = {
                        "Connected Components": [clean_name(comp1), clean_name(comp2)],
                        "Transformation": {
                            "Joint Origin": [
                                joint_origin.x,
                                joint_origin.y,
                                joint_origin.z
                            ],
                            "Joint Axis": [
                                joint_axis.x,
                                joint_axis.y,
                                joint_axis.z
                            ]
                        }
                    }
                    return joint_info

                except:
                    ui.messageBox(f'Failed to extract joint for components: {comp1}, {comp2}. . Most likely your joint is not defined correctly.')
                    return None
            return None
            
        for occ in allOccs:
            entry = extract_component_data(occ)
            if entry:
                output_data["Components"].append(entry)

        extracted_joint_num = 0

        for joint in joints:
            entry = extract_joint_data(joint)
            if entry:
                extracted_joint_num += 1
                output_data["Joints"].append(entry)
                
        print(f"Extracted: {extracted_joint_num} joints")

        output_file_path = os.path.join(output_folder, 'fusion_info.json')
        with open(output_file_path, 'w') as output_file:
            json.dump(output_data, output_file, indent=4)

        ui.messageBox(f'Output saved to: {output_file_path}. Exported {len(output_data["Components"])} components and {len(output_data["Joints"])} joints.')

    except Exception as e:
        if ui:
            ui.messageBox(f'Failed: {traceback.format_exc()}')

# Helper function to convert a rotation matrix to a quaternion
def rotation_matrix_to_quaternion(R):
    import math
    trace = R[0][0] + R[1][1] + R[2][2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2][1] - R[1][2]) * s
        y = (R[0][2] - R[2][0]) * s
        z = (R[1][0] - R[0][1]) * s
    elif (R[0][0] > R[1][1]) and (R[0][0] > R[2][2]):
        s = 2.0 * math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2])
        w = (R[2][1] - R[1][2]) / s
        x = 0.25 * s
        y = (R[0][1] + R[1][0]) / s
        z = (R[0][2] + R[2][0]) / s
    elif R[1][1] > R[2][2]:
        s = 2.0 * math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2])
        w = (R[0][2] - R[2][0]) / s
        x = (R[0][1] + R[1][0]) / s
        y = 0.25 * s
        z = (R[1][2] + R[2][1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1])
        w = (R[1][0] - R[0][1]) / s
        x = (R[0][2] + R[2][0]) / s
        y = (R[1][2] + R[2][1]) / s
        z = 0.25 * s
    return w, x, y, z