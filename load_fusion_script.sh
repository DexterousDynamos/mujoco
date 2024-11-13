FUSION_BASE_PATH_MAC="/Users/$(whoami)/Library/Application Support/Autodesk/Autodesk Fusion 360/API/Scripts/"
SCRIPT_FOLDER="ExtractFusionInfo"
SCRIPT_PATH_MAC="${FUSION_BASE_PATH_MAC}${SCRIPT_FOLDER}"
MUJOCO_REPO_PATH=$(pwd)

if [ -d "$FUSION_BASE_PATH_MAC" ]; then
    if [ ! -d "$SCRIPT_PATH_MAC" ]; then
        mkdir -p "$SCRIPT_PATH_MAC"
        echo "Created ExtractFusionInfo folder at $SCRIPT_PATH_MAC"
    else
        echo "ExtractFusionInfo folder already exists at $SCRIPT_PATH_MAC. Overwriting..."
    fi
    echo "Fusion 360 path found at $SCRIPT_PATH_MAC"
    cp fusion/ExtractFusionInfo.py "$SCRIPT_PATH_MAC"
    echo "Copied ExtractFusionInfo to Fusion 360 path"
    cp fusion/ExtractFusionInfo.manifest "$SCRIPT_PATH_MAC"
    echo "Copied ExtractFusionInfo.manifest to Fusion 360 path"
    touch "$SCRIPT_PATH_MAC/MUJOCO_REPO_PATH.txt"
    echo "$MUJOCO_REPO_PATH" > "$SCRIPT_PATH_MAC/MUJOCO_REPO_PATH.txt"
else
    echo "Fusion 360 path not found. Please manually copy the ExtractFusionInfo folder to the Fusion 360 scripts path."
    exit 1  
fi