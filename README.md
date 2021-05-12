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
Points of permanent delineators from the HD map mapped to pixel locations (green) and points without known corresponding pixels (red) rendered by a poorly calibrated camera model.
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

### See below for the necessary dependencies.

### Compiling

```shell
mkdir build && cd build
cmake ..
cmake --build . -j8
```

#### CMake Configure Options

```shell
# Default is OFF for all options

-DWITH_TESTS=ON/OFF     # Build with tests. 

-DWITH_OPENCV=ON/OFF    # Build with OpenCV. 
                        # Set to ON to render the projected world objects during optimization. 

```

### Running

```shell
./app/StaticCalibration -h 
```

### Evaluating

```shell
./evaluate/EvaluateStaticCalibration -h   # Prints the help message. See it for the required input data.g
```

### Testing

```shell
ctest --verbose
```

***

# Input Data

The calibration requires a 2D-3D mapping from pixels to objects to estimate the 6DoF pose of the camera.

### Objects

```shell
-o [ --objects_file ] <filename>
```

The `objects_file` is required to be in the YAML format as specified:

```yaml
objects:
  - # Must match with the ids in the pixel file. This establishes the mapping.
    id: 4008327
    # Has to be pole. No other type is currently implemented.
    type: pole
    # Has to be permanentDelineator. No other name is currently implemented.
    name: permanentDelineator
    # [XYZ | East/North/Height] coordinates in a right-handed coordinate system. 
    utm_coord: [ 692571.92953127041, 5339110.7990111383, 549.29530761491162 ]
    # The height in the same coordinate system as in utm_coord.
    height: 1.135
  - …
```

To facilitate the creation of the `objects_file`, we have released an [OpenDRIVE](https://github.com/Brucknem/OpenDRIVE)
parser that converts from the OpenDRIVE V1.4 standard to our internally used format.  
See the [README](https://github.com/Brucknem/OpenDRIVE/blob/main/README.md) in the project for the usage.

### Pixels

```shell
-p [ --pixels_file ] <filename>
```

The `pixels_file` is required to be in the YAML format as specified:

```yaml
- # Must match with the ids in the objects_file. This establishes the mapping.
  id: 4008327
  # The UV pixel coordinates of the object.
  pixels:
    - [ 13, 373 ]
    - [ 13, 374 ]
    - [ 14, 374 ]
    - [ 13, 375 ]
    - [ 14, 375 ]
    - …
- …
```

To facilitate the creation of the `pixels_file`, we have released
an [Annotation Tool](https://github.com/Brucknem/DataAnnotationTools)
that can be used to mark pixels in a frame and outputs them in the required format.  
See the [README](https://github.com/Brucknem/DataAnnotationTools/blob/main/README.md) in the project for the usage.

***

# Dependencies

## External Dependencies

`# Only necessary if compiling with -DWITH_OPENCV=ON`

These dependencies have to be installed on your system. Follow their instructions on how to install them.

- [CUDA](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html) - You have to install CUDA and the
  Nvidia drivers according to your GPU.

There is no way to automate this proces as it is dependend on your system.

***

- [CMake](https://cmake.org/) - For building the libraries
- [Ceres Solver](http://ceres-solver.org/) - The non-linear solver for the static calibration
- [OpenCV](https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html) - With CUDA support for the dynamic
  stabilization `# Only necessary if compiling with -DWITH_OPENCV=ON`

To facilitate the setup of CMake, Ceres and OpenCV you can use the [setup script](/extern/setup_ceres_opencv.sh). This
should install all necessary dependencies and pull & compile the libraries from source. This might be outdated by now,
but I will update the script in the next days.  
If compile errors arise, the CMake output is a good start to debug.

- [Boost](https://www.boost.org/) - For parsing the command line options

## Internal Dependencies

These dependencies are pulled by CMake when the project is built. You `do not` have to install them manually.

- [GoogleTest](https://github.com/google/googletest) `# Only built if compiling with -DWITH_TESTS=ON`
- [YAML-CPP](https://github.com/jbeder/yaml-cpp.git)
