#!/bin/bash

# Define colors
GREEN="\033[0;36m"          # Green
ORANGE="\033[0;33m"         # Orange
BOLD_RED="\033[1;31m"       # Bold Red
NC="\033[0m"                # No Color (reset)

# Loop through all .xml files in the current directory
for file in *.xml; do
    # Check if file exists (in case there are no .xml files)
    if [[ -f "$file" ]]; then
        # Check if the file name starts with "exclude"
        if [[ ! "$file" =~ ^exclude ]]; then
            echo -e "${GREEN}Running Mujoco viewer for:\t${NC} $file"
            python3 -m mujoco.viewer --mjcf="$file" &
        else
            echo -e "${ORANGE}Skipping excluded file:\t\t${NC} $file"
        fi
    else
        echo -e "${BOLD_RED}No xml files exist in the current directory. Exiting...${NC}"
    fi
done
