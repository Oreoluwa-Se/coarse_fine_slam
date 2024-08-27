#!/bin/bash

# Default build type
BUILD_TYPE=${1:-Release}
USE_RVIZ=${2:-false}
BAG_LOCATION=${3:-/mnt/c/wslShared/robotics/lidar/coarse_fine_slam/bag/rad_test_1.bag}

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
  roslaunch coarse_fine_slam hub.launch run_rviz:=$USE_RVIZ bag_file_location:=$BAG_LOCATION >first_trial.txt
elif [ "$BUILD_TYPE" == "Debug" ]; then
  # Launch the ROS node with the specified build type and RVIZ option
  roslaunch coarse_fine_slam hub_debug.launch run_rviz:=$USE_RVIZ bag_file_location:=$BAG_LOCATION >first_trial.txt
else
  echo "Unknown build type: $BUILD_TYPE. Exiting."
  exit 1
fi
