#!/bin/bash

# Default build type
BUILD_TYPE=${1:-Release}
USE_RVIZ=${2:-false}
BAG_LOCATION=${3}

# Ensure BAG_LOCATION is provided
if [ -z "$BAG_LOCATION" ]; then
  echo "Error: BAG_LOCATION is required. Please provide the path to the .bag file as the third argument."
  echo "Usage: ./run.sh [BUILD_TYPE] [USE_RVIZ] [BAG_LOCATION]"
  echo "Example: ./run.sh Release true /path/to/your/file.bag"
  exit 1
fi

# Get the current working directory
CURRENT_DIR=$(pwd)

# Navigate to the root of the workspace (assuming the script is run from src or a subdirectory)
if [[ "$CURRENT_DIR" == */src* ]]; then
  WORKSPACE_ROOT=$(dirname "$CURRENT_DIR")
else
  WORKSPACE_ROOT="$CURRENT_DIR"
fi

# Echo the workspace root to verify it's correct
echo "Workspace root detected as: $WORKSPACE_ROOT"

# Verify that we're in the root of a catkin workspace
if [ ! -f "$WORKSPACE_ROOT/src/CMakeLists.txt" ]; then
  echo "Error: No CMakeLists.txt found in the src/ directory. Are you sure this is a catkin workspace?"
  exit 1
fi

# Check if the map_storage library exists in the libs folder
MAP_STORAGE_DIR="$WORKSPACE_ROOT/libs/map_storage"
if [ ! -d "$MAP_STORAGE_DIR" ]; then
  echo "map_storage library not found in $WORKSPACE_ROOT/libs. Cloning the repository..."
  
  # Create the libs directory if it doesn't exist
  mkdir -p "$WORKSPACE_ROOT/libs"
  
  # Clone the map_storage repository
  git clone https://github.com/Oreoluwa-Se/map_storage.git "$MAP_STORAGE_DIR"
  
  # Check if the clone was successful
  if [ $? -ne 0 ]; then
    echo "Failed to clone map_storage repository. Exiting."
    exit 1
  fi
else
  echo "map_storage library already exists in $WORKSPACE_ROOT/libs."
fi

# Navigate to the workspace root
cd "$WORKSPACE_ROOT"

# Run catkin_make with the specified build type
clear && catkin_make -DCMAKE_BUILD_TYPE=$BUILD_TYPE

# Check if catkin_make was successful
if [ $? -ne 0 ]; then
  echo "catkin_make failed. Exiting."
  exit 1
fi

# Source the setup.bash file to set up the environment
clear && source devel/setup.bash

# Launch the ROS node with the specified build type and RVIZ option
if [ "$BUILD_TYPE" == "Release" ]; then
  # Launch the ROS node with the specified build type and RVIZ option
  roslaunch coarse_fine_slam hub.launch run_rviz:=$USE_RVIZ bag_file_location:=$BAG_LOCATION >first_trial_full.txt
elif [ "$BUILD_TYPE" == "Debug" ]; then
  # Launch the ROS node with the specified build type and RVIZ option
  roslaunch coarse_fine_slam hub_debug.launch run_rviz:=$USE_RVIZ bag_file_location:=$BAG_LOCATION >first_trial.txt
else
  echo "Unknown build type: $BUILD_TYPE. Exiting."
  exit 1
fi
