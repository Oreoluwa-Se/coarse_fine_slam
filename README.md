# Coarse-Fine SLAM

This project implements a SLAM algorithm designed to provide robust and efficient odometry for both indoor and outdoor environments. The front-end utilizes an invariant Kalman filter for real-time state estimation, while the back-end employs a pose-graph to smooth and refine the estimated trajectory.

## Point Storage

The point storage system is based on the [map_storage](https://github.com/Oreoluwa-Se/map_storage) library, which utilizes shallow trees for efficient insertion, deletion, and searching. This approach offers performance comparable to, or better than, incremental kd-trees. Detailed information and benchmarks can be found in the repository.

## LiDAR Algorithm

The LiDAR processing algorithm is inspired by the paper ["An Incremental Right-Invariant Kalman Filter for LiDAR/Camera/IMU Odometry"](https://arxiv.org/pdf/2402.05003), which introduces a novel method for fusing LiDAR, camera, and IMU data for odometry. Our implementation extends these ideas, combining them with techniques from the Fast-LIO2 paper to optimize for various scenarios. Key features include:

- **Coarse-to-Fine ICP:** Implements a hierarchical approach to Iterative Closest Point (ICP) to speed up convergence.
- **Double Downsampling:** Utilizes a double downsampling strategy from the Kiss-ICP paper to reduce computational load.
- **Adaptive Radius:** Adjusts the search radius dynamically based on a window of past overlaps, smoothing out fluctuations.
- **Outlier Rejection:** Incorporates robust techniques to estimate measurement weights and filter out unreliable points.
- **Averaged Process Covariance:** Smooths out the process covariance (Q-matrix) over time using a sliding window, reducing the impact of uncertain measurements.

While the Kalman filter component of the algorithm is complete, ongoing work includes:

- **Pose-Graph Backend:** Implementing pose-graph optimization for key-frame-based smoothing.
- **Loop Closure:** Developing a loop closure algorithm to correct drift over long trajectories.

## Dependencies

This project has been tested on Ubuntu 18.04 and 20.04. The following libraries are required:

1. C++14 or higher
2. Eigen
3. TBB (Threading Building Blocks)
4. Boost
5. Sophus
6. [map_storage](https://github.com/Oreoluwa-Se/map_storage) (place in the `libs` folder)

## Compilation and Running the Program

Parameters can be configured in the config/params.yaml file
Debug Mode:

```sh
./run.sh Debug

```

Release Mode:

```sh
./run.sh Release
```

@misc{CoarseFineSlam2024,
  author       = {Oreoluwa Seweje},
  title        = {Coarse Fine SLAM},
  year         = {2024},
  url          = {<https://github.com/Oreoluwa-Se/coarse_fine_slam>},
  note         = {An Adaptable Visual Odometry System},
}
