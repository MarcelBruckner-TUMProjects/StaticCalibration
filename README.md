# Automated Camera Stabilization and Calibration for Intelligent Transportation Systems - Static Calibration

- [See the project website of Providentia++](https://innovation-mobility.com/en)
- [See the overview repository for general information about my Guided Research](https://github.com/Brucknem/GuidedResearch)
- [See the website for information and evaluation results of the project](https://brucknem.github.io/StaticCalibration)

*** 

# Introduction

We track and predict the real-world location of vehicles in the test area to pass this information to the drivers and
autonomous vehicles. To accurately predict the locations the system needs to be calibrated precisely towards a global
reference frame. This is a time consuming process that often has to be done by hand. The cameras translational,
rotational and intrinsic parameters decalibrate over time due to environmental influences on the mounting constructions
and gantry bridges as well as from the natural wear of the materials.

Within the project high definition maps (HD maps) of the enclosed environment are used extensively. These HD maps offer
approximations of the real-world positions of the highway lanes, the gantry bridges, objects like poles and permanent
delineators and traffic signals like speed limits or exit markers. We use this spatial information and a mapping from
the objects to pixels in the video frame to solve a Bundle Adjustment (BA) problem by minimizing the reprojection-error.
We jointly optimize for the cameras intrinsic and extrinsic parameters as well as the real-world locations to recover
the camera poses from the observations.

[Please see the report for in depth information about the pose estimation procedure](https://github.com/Brucknem/GuidedResearch/blob/main/report/report.pdf)

***

### Before calibration
![The uncalibrated camera pose.](https://github.com/Brucknem/GuidedResearch/blob/main/report/images/calibration/background_uncalibrated_with_mapping.png?raw=true)
<p align="center">
Points of permanent delineators mapped to pixel locations (green) and points without known corresponding pixels (red) rendered by a poorly calibrated camera model.
The mapping from the projected points to their expected pixels is drawn in light blue.
</p>

### After calibration
![The calibrated camera pose.](https://github.com/Brucknem/GuidedResearch/blob/main/report/images/calibration/background_calibrated.png?raw=true)
<p align="center">
The same points after the calibration procedure.
The rendered positions of the mapped points align with their respective pixels.
The drawn mapping disappears as the distances approach 0.
</p>
***

# Running in Standalone Mode

***

# Dependencies

## External Dependencies

These dependencies have to be installed on your system. Follow their instructions on how to install them.

- [CUDA](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html) - You have to install CUDA and the
  Nvidia drivers according to your GPU.

There is no way to automate this proces as it is dependend on your system.

***

- [CMake](https://cmake.org/) - For building the libraries
- [Ceres Solver](http://ceres-solver.org/) - The non-linear solver for the static calibration
- [OpenCV](https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html) - With CUDA support for the dynamic
  stabilization

To facilitate the setup of CMake, Ceres and OpenCV you can use the [setup script](../main/setup_ceres_opencv.sh). This
should install all necessary dependencies and pull & compile the libraries from source. This might be outdated by now.
If compile errors arise, the CMake output is a good start to debug.

- [Boost](https://www.boost.org/)

## Internal Dependencies

These dependencies are pulled by CMake when the project is built. You `do not` have to install them manually.

- [GoogleTest](https://github.com/google/googletest)
- [YAML-CPP](https://github.com/jbeder/yaml-cpp.git)
