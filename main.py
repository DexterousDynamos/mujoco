from src.Fusion_to_Mujoco import Fusion_to_Mujoco

if __name__ == "__main__":
    model = Fusion_to_Mujoco(reduce_stls=True, use_rel_stlpath=True)
    model.copy_assets()
    model.export_xml()
    model.run_interactive()