cmake_minimum_required(VERSION 3.10)

########################################################################################################################
### Includes ###
########################################################################################################################

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/extern)

########################################################################################################################
### App ###
########################################################################################################################

add_executable(StaticCalibration StaticCalibration.cpp)
target_link_libraries(StaticCalibration StaticCalibration-lib Eigen3::Eigen yaml-cpp)
target_include_directories(StaticCalibration PUBLIC "${PROJECT_BINARY_DIR}")
