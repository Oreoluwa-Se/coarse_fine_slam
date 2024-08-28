# Coarse-Fine SLAM

Coarse-Fine SLAM is a robust and efficient SLAM algorithm designed for both indoor and outdoor environments. The algorithm combines an invariant Kalman filter for real-time state estimation in the front-end with a pose-graph in the back-end for trajectory smoothing and refinement.

## Overview

### Point Storage

The point storage system leverages the [map_storage](https://github.com/Oreoluwa-Se/map_storage) library, which implements shallow trees for efficient operations like insertion, deletion, and searching. This system offers performance comparable to, or better than, incremental kd-trees. Detailed benchmarks and further information are available in the repository.

### LiDAR Processing Algorithm

Our LiDAR processing algorithm extends ideas from several state-of-the-art papers, resulting in a powerful and flexible system for SLAM:

- **Coarse-to-Fine ICP:** A hierarchical approach to Iterative Closest Point (ICP) that accelerates convergence.
- **Double Downsampling:** Implements a double downsampling strategy inspired by the [Kiss-ICP](https://arxiv.org/pdf/2209.15397) paper, reducing computational load.
- **Adaptive Radius:** Dynamically adjusts the search radius based on a window of past overlaps to smooth out fluctuations.
- **Outlier Rejection:** Uses robust techniques to estimate measurement weights and filter out points with uncertain neighbourhood information.
- **Averaged Process Covariance:** Smooths the process covariance (Q-matrix) over time using a sliding window, Allowing for more stable noise adjustment.
- **IMU Prediction:** Following insights from the [Point-LIO](https://onlinelibrary.wiley.com/doi/epdf/10.1002/aisy.202200459) paper, IMU readings are predicted as part of the state, allowing for intrinsic noise handling.
- **Online Calibration:** [OPTIONAL] The algorithm is designed to be adaptable, optionally accounting for disruptions to relative sensor locations. Eg. Bumpy environments e.t.c

### Ongoing Work

- **Pose-Graph Backend:** Implementing key-frame-based pose-graph optimization for smoother trajectories.
- **Loop Closure:** Developing a loop closure algorithm to correct drift when recognizing previously visited locations.

## Dependencies

This project has been tested on Ubuntu 18.04 and 20.04. The following dependencies are required:

1. C++14 or higher
2. Eigen
3. TBB (Threading Building Blocks)
4. Boost
5. Sophus
6. [map_storage](https://github.com/Oreoluwa-Se/map_storage) (place in the `libs` folder)

Dependencies 2-5 will be installed via CMake if not already installed. The `run.sh` file would install required dependencies.

## Compilation and Execution

### Configuration

Parameters can be configured in the `config/params.yaml` file.

### Script Inputs

The script requires three inputs:

1. **BUILD_TYPE**: Specifies the type of build.
    - `Debug`: For debugging.
    - `Release`: For optimized performance.
    - `RelWithDebInfo`: A mix of Release and Debug.

2. **USE_RVIZ**: Determines whether RViz, the 3D visualization tool, should be launched after building.
    - `true`: Launch RViz.
    - `false`: Do not launch RViz.

3. **BAG_LOCATION**: The full path to the ROS `.bag` file containing the data.

### Running the Script

To execute the script, use the following command:

```bash
chmod +x run.sh
./run.sh BUILD_TYPE USE_RVIZ BAG_LOCATION
```

## Citation

if you use this project in your work please citer as follows:

```
@misc{CoarseFineSlam,
  author       = {Oreoluwa Seweje},
  title        = {Merging Ideas for Lidar-Imu Odometry},
  year         = {2024},
  url          = {<https://github.com/Oreoluwa-Se/coarse_fine_slam}>,
  note         = {Adaptable Slam Algorithm. Merging Ideas from Existing Papers.},
}
```
