import os
from utils import reduce_mesh

def main():
    input_file = "../assets/fusion_export_2024-11-19_14-51-21/Carpals.stl"
    output_file = "output_Carpals.stl"
    reduction_level = "extremely_low" # "medium"
    
    
    reduce_mesh(input_file, output_file, reduction_level, verbose=True)
        
    # if os.path.isfile(output_file):
    #     output_size = os.path.getsize(output_file)
    #     print(f"Output file size: {output_size} bytes")
    # else:
    #     print(f"Error: The output file {output_file} was not created.")

if __name__ == "__main__":
    main()