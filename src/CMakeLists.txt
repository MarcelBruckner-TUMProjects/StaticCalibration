########################################################################################################################
### YAML CPP ###
########################################################################################################################

# Download and unpack yaml-cpp at configure time
configure_file(CMakeLists.txt.in yaml-cpp-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
        RESULT_VARIABLE result
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/yaml-cpp-download)
if (result)
    message(FATAL_ERROR "CMake step for yaml-cpp failed: ${result}")
endif ()
execute_process(COMMAND ${CMAKE_COMMAND} --build .
        RESULT_VARIABLE result
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/yaml-cpp-download)
if (result)
    message(FATAL_ERROR "Build step for yaml-cpp failed: ${result}")
endif ()

# Add yaml-cpp directly to our build.
add_subdirectory(${CMAKE_CURRENT_BINARY_DIR}/yaml-cpp-src
        ${CMAKE_CURRENT_BINARY_DIR}/yaml-cpp-build
        EXCLUDE_FROM_ALL)

########################################################################################################################
### Library ###
########################################################################################################################

if (WITH_OPENCV)
    find_package(OpenCV REQUIRED)
    include_directories(${OpenCV_INCLUDE_DIRS})

    list(APPEND EXTRA_LIBS ${OpenCV_LIBS})
endif ()

find_package(Ceres REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(Boost REQUIRED COMPONENTS program_options REQUIRED filesystem REQUIRED date_time)
include_directories(${Boost_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/include)

########################################################################################################################
### Static Calibration ###
########################################################################################################################

add_library(StaticCalibration-lib
        CameraPoseEstimation.cpp
        camera/RenderingPipeline.cpp
        residuals/DistanceFromIntervalResidual.cpp
        residuals/DistanceResidual.cpp
        residuals/CorrespondenceResidual.cpp
        objects/ParametricPoint.cpp
        objects/WorldObject.cpp
        objects/ObjectsLoading.cpp
        objects/YAMLExtension.cpp
        utils/CommandLineParser.cpp
        )

target_include_directories(StaticCalibration-lib
        INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
        PRIVATE ${CMAKE_CURRENT_BINARY_DIR}
        PUBLIC "${PROJECT_BINARY_DIR}"
        )

target_link_libraries(StaticCalibration-lib
        Eigen3::Eigen
        Ceres::ceres
        yaml-cpp
        ${Boost_LIBRARIES}
        ${EXTRA_LIBS}
        )
