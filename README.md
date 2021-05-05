# Automated Camera Stabilization and Calibration for Intelligent Transportation Systems

[See the website for information and evaluation results of the project](https://brucknem.github.io/GuidedResearch)

*** 

## Background
Within the [PROVIDENTIA](https://www.bmvi.de/SharedDocs/DE/Artikel/DG/AVF-projekte/providentia-plusplus.html) project, a section of the highway A9 between Munich and Nuremberg was converted to a testing site for autonomous driving. As part of this, a large sensor network system has been set up along the highway to allow monitoring and steering of traffic as well as to improve the coordination between autonomous and traditional cars. The primary task of the intelligent system is to create a digital traffic twin that accurately represents the physical road situation in real-time. Based on this digital twin, the smart infrastructure can provide a far-reaching and comprehensive view to the drivers and autonomous cars in order to improve their situational awareness within the current traffic environment. A video about the PROVIDENTIA project is available on https://youtu.be/4oCnQlGFuc4.

## Description
A key challenge lies in the reliable and accurate calibration of the different sensors. The calibration is especially challenging when the sensor is subject to real-life disturbances like vibration of its mounting pole caused by wind or displacements due to temperature expansion. The aim of this Master’s thesis is to investigate the feasibility to automatically stabilize and calibrate a shaking camera using an additional IMU sensor that delivers measurements on the disturbances.

## Tasks
- Familiarization with stabilization and calibration methods via literature research
- Development of an experimental setup consisting of an oscillating platform equipped with a camera and an IMU
- Development of an approach to automatically stabilize and calibrate the camera
- Evaluation of the concept using real-life data

***

## [Implementation](https://github.com/Brucknem/GuidedResearch/tree/main/implementation)

## [Literature](https://github.com/Brucknem/Graduation/tree/main/literature)

***

# Dependencies

## External Dependencies

These dependencies have to be installed on your system. Follow their instructions on how to install them. 

- [CUDA](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html) - You have to install CUDA and the Nvidia drivers according to your GPU.

There is no way to automate this proces as it is dependend on your system.

***

- [CMake](https://cmake.org/) - For building the libraries
- [Ceres Solver](http://ceres-solver.org/) - The non-linear solver for the static calibration
- [OpenCV](https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html) - With CUDA support for the dynamic stabilization

To facilitate the setup of CMake, Ceres and OpenCV you can use the [setup script](../main/setup_ceres_opencv.sh). This should install all necessary dependencies and pull & compile the libraries from source. 
This might be outdated by now. If compile errors arise, the CMake output is a good start to debug.

- [Boost](https://www.boost.org/)

## Internal Dependencies

These dependencies are pulled by CMake when the project is built. You `do not` have to install them manually.

- [GoogleTest](https://github.com/google/googletest)
- [YAML-CPP](https://github.com/jbeder/yaml-cpp.git)
