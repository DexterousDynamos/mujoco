import numpy as np
from stl import mesh
import trimesh
import os
import datetime
import re

def reduce_mesh(input_file, output_file, quality_after, verbose: bool=False):#
    def convert_stl_to_trimesh(stl_mesh):
        # Extract vertices and faces from the STL mesh
        vertices = stl_mesh.vectors.reshape(-1, 3)  # Flatten the triangle vertices
        # Deduplicate vertices
        vertices, unique_indices = np.unique(vertices, axis=0, return_inverse=True)
        # Create faces using unique indices
        faces = unique_indices.reshape(-1, 3)
        # Create a Trimesh object
        return trimesh.Trimesh(vertices=vertices, faces=faces, process=True)

    if not os.path.isfile(input_file):
        raise FileNotFoundError(f"The file {input_file} does not exist.")

    # Load the STL file
    original_mesh = mesh.Mesh.from_file(input_file)
    
    # Convert to trimesh object
    trimesh_mesh = convert_stl_to_trimesh(original_mesh)

    if verbose:
        # os.path.getsize(input_file)
        print(f"Input file size: {os.path.getsize(input_file)} bytes")
        print("Faces before:", len(trimesh_mesh.faces))

    # Determine the reduction factor
    if quality_after == 'extremely_low':
        factor = 0.1
    if quality_after == 'low':
        factor = 0.3
    elif quality_after == 'medium':
        factor = 0.5
    elif quality_after == 'high':
        factor = 0.7
    elif quality_after == 'extremely_high':
        factor = 0.9
    else:
        raise ValueError("Reduction level must be 'extremely_low', 'low', 'medium', 'high', or 'extremely_high'.")
    
    # Calculate target reduction as a fraction
    if verbose:
        print(f"Target reduction: {factor * 100:.2f}%")
    
    # Simplify the mesh
    simplified_mesh = trimesh_mesh.simplify_quadric_decimation(factor)
    
    # Validate simplified mesh
    if len(simplified_mesh.faces) == 0 or len(simplified_mesh.vertices) == 0:
        raise ValueError("Simplified mesh is empty. Check your reduction parameters.")
    
    if verbose:
        print("Faces after:", len(simplified_mesh.faces))

    # Save the simplified mesh
    simplified_mesh.export(output_file)
    if verbose:
        print(f"Mesh reduced and saved to {output_file}")
        print(f"Output file size: {os.path.getsize(output_file)} bytes")

def find_latest_folder(asset_folder: str) -> str:
    '''
    Find the latest folder in the asset folder.

    Args:
        asset_folder (str): The path to the asset folder

    Returns:
        str: The path to the latest folder.
    '''
    # base_path = self.asset_folder
    # Regular expression to match folders in the format "fusion_export_YYYY-MM-DD_HH-MM-SS"
    folder_pattern = re.compile(r"fusion_export_(\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})")

    latest_time = None
    latest_folder = None

    # Iterate over items in the base directory
    for folder_name in os.listdir(asset_folder):
        folder_path = os.path.join(asset_folder, folder_name)
        
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