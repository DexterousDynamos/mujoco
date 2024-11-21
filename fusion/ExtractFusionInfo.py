import adsk.core, adsk.fusion, traceback
import re
import os
import json
from adsk.fusion import Occurrence
import datetime
import hashlib


def hash_string(s: str, length: int = 8):
    '''
    Hashes a string using SHA256 and returns the first `length` characters.
    '''
    return hashlib.sha256(s.encode()).hexdigest()[:length]

def get_unique_id_from_occurence(occ, length: int = 8):
    '''
    Takes an occurrence or string as input, then gets the 'unique' path name
    and computes the unique ID by hashing it.
    The output hash is truncated to the specified length.
    '''
    if isinstance(occ, Occurrence):
        unique_name = occ.fullPathName
        hashed_name = hash_string(unique_name)
        return hashed_name[:length]
    elif isinstance(occ, str):
        return hash_string(occ)[:length]
    return None

def get_unique_parent_id_from_occurence(occ: Occurrence, length: int = 8):
    '''
    Takes an occurrence as input, then gets the 'unique' path name
    of it's parent and computes the unique ID by hashing it.
    The output hash is truncated to the specified length.
    '''
    plus_index = occ.fullPathName.rfind("+")
    if plus_index == -1:
        unique_name = 'Root'
    else:
        parent_name = occ.fullPathName[:plus_index]
        unique_name = hash_string(parent_name)
    return unique_name

def clean_name(name):
    '''
    Cleans up the name by removing the version info but keeping the occurence number.
    '''
    match = re.match(r'(\S+)\s.*:(\d+)', name)
    if match:
        return f"{match.group(1)}:{match.group(2)}"
    return name  

def get_parent_name(occ: Occurrence):
    '''
    Extracts the parent name and the unique name from the occurrence.
    '''
    plus_index = occ.fullPathName.rfind("+")
    if plus_index == -1:
        parent_name = 'Root'
        unique_name = occ.fullPathName
    else:
        parent_name = occ.fullPathName[:plus_index]
        unique_name = occ.fullPathName[plus_index+1:] 
    
    return parent_name, unique_name  #

def get_base_name(name):
    '''
    Cleans up the name by removing the version info and the occurence number.
    This is also the exported STL file name.
    '''
    base_name = re.sub(r'(\s*v\s*\d+:\d+|:\d+)', '', name).strip()
    return base_name

def is_desired_component(occ, desired_component_names):
    '''
    Checks if the component name is in the desired component names list.
    '''
    if isinstance(occ, Occurrence): 
        base_name = get_base_name(occ.name)
    elif isinstance(occ, str):
        base_name = get_base_name(occ)
    else: 
        return False
    return base_name in desired_component_names

def truncate_occurence_path(occ):
    '''
    Truncates the occurence path to the last occurence name.
    '''
    if isinstance(occ, Occurrence):
        plus_index = occ.fullPathName.rfind("+")
        if plus_index == -1:
            return occ.fullPathName
        return occ.fullPathName[:plus_index]
    elif isinstance(occ, str):
        plus_index = occ.rfind("+")
        if plus_index == -1:
            return occ
        return occ[:plus_index]
    return None

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

        # Set the desired component names, already in hierachical order
        desired_component_names = [
            'Tower_MK3', 'Carpals','M-AP','T-AP','T-AP_OUT', 'M-PP', 'I-PP', 'P-PP' , 'T-PP' ,'M-MP','M-DP','T-DP', 'P-Assembly', 'I-Assembly', 'M-Assembly', 'T-Assembly'
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
            "components": [],
            "joints": [],
            "desired_component_names": desired_component_names,
        }
        
        # Root component and all occurrences
        rootComp = design.rootComponent
        allOccs = rootComp.allOccurrences
        joints = rootComp.allJoints
        
        # define list of all id's for joints later
        component_id_stl_dict = {}
        
        def extract_component_data(occ: Occurrence):
            occ_name = occ.name  # Includes version (e.g., "M_MP v76:1")
            base_name = get_base_name(occ_name)

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
            translation = (transform.getCell(0, 3) / 100, transform.getCell(1, 3) / 100, transform.getCell(2, 3) / 100)        
            # Set up the STL export
            export_mgr = app.activeProduct.exportManager
            stl_export_options = export_mgr.createSTLExportOptions(occ)
            stl_export_options.isBinaryFormat = True  # Set binary STL format
            stl_filename = f"{base_name}.stl"  # Filename only
            stl_file_path = os.path.join(output_folder, stl_filename)  # Save in output directory
            stl_export_options.filename = stl_file_path
            export_mgr.execute(stl_export_options)
            
            # save for later use in joints
            component_id_stl_dict[get_unique_id_from_occurence(occ)] = base_name

            entry = {
                "component": {
                    "name": base_name,
                    "id": get_unique_id_from_occurence(occ)
                },
                "parent": {
                    "id": get_unique_parent_id_from_occurence(occ),
                },
                "transformation": {
                    # "Rotation Matrix": rotation_matrix,
                    "quaternion": quaternion,
                    "translation": translation
                },
                "stl_file": stl_filename,  # Add STL file path to the component data
                "is_base_component": "Assembly" not in base_name
            }
            return entry
        
        def extract_joint_data(joint):
            if joint.jointMotion.jointType == 1: # revolute joint
                occ1, occ2 = joint.occurrenceOne, joint.occurrenceTwo
                occ1_full_path, occ2_full_path = occ1.fullPathName, occ2.fullPathName
              
                # Delete subcomponents, to reach the parent component which was used to generate the stl files
                search_index = 0
                while not get_unique_id_from_occurence(occ1_full_path) in component_id_stl_dict:
                    occ1_full_path = truncate_occurence_path(occ1)
                    search_index += 1
                    if search_index > 10:
                        ui.messageBox(f'Failed to find STL file for component: {occ1}.')
                search_index = 0
                while not get_unique_id_from_occurence(occ2_full_path) in component_id_stl_dict:
                    occ2_full_path = truncate_occurence_path(occ2)
                    search_index += 1
                    if search_index > 10:
                        ui.messageBox(f'Failed to find STL file for component: {occ2}.')
            
                id_1 = get_unique_id_from_occurence(occ1_full_path)
                id_2 = get_unique_id_from_occurence(occ2_full_path)
                
                base_name1 = component_id_stl_dict[id_1]
                base_name2 = component_id_stl_dict[id_2]
                
                # Ensure that the base name is sorted correctly
                if desired_component_names.index(base_name1) > desired_component_names.index(base_name2):
                    occ1, occ2 = occ2, occ1
                    base_name1, base_name2 = base_name2, base_name1
                    id_1, id_2 = id_2, id_1
                                
                try:
                    joint_origin = joint.geometryOrOriginOne.origin
                    joint_axis = joint.jointMotion.rotationAxisVector
                    joint_info = {
                        "component_base": {
                            "id": id_1,
                            "name": base_name1                            
                        },
                        "component_rotating": {
                            "id": id_2,
                            "name": base_name2
                        },
                        "transformation": {
                            "joint_origin": [
                                joint_origin.x / 100 ,
                                joint_origin.y / 100,
                                joint_origin.z / 100
                            ],
                            "joint_axis": [
                                joint_axis.x,
                                joint_axis.y,
                                joint_axis.z
                            ],
                            "joint_range": [
                                joint.jointMotion.rotationLimits.minimumValue,
                                joint.jointMotion.rotationLimits.maximumValue
                            ]
                        }
                    }
                    return joint_info

                except:
                    ui.messageBox(f'Failed to extract joint for components: {occ1}, {occ2}. . Most likely your joint is not defined correctly.')
                    return None
            return None
            
        for occ in allOccs:
            entry = extract_component_data(occ)
            if entry:
                output_data["components"].append(entry)

        extracted_joint_num = 0

        print("Extracting joints...")
        for joint in joints:
            entry = extract_joint_data(joint)
            if entry:
                extracted_joint_num += 1
                output_data["joints"].append(entry)
                
        print(f"Extracted: {extracted_joint_num} joints")

        output_file_path = os.path.join(output_folder, 'fusion_info.json')
        with open(output_file_path, 'w') as output_file:
            json.dump(output_data, output_file, indent=4)

        ui.messageBox(f'Output saved to: {output_file_path}. Exported {len(output_data["components"])} components and {len(output_data["joints"])} joints.')

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