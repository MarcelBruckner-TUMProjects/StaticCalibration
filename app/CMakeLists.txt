cmake_minimum_required(VERSION 3.10)

########################################################################################################################
### Packages ###
########################################################################################################################

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(${PROJECT_SOURCE_DIR}/include)

########################################################################################################################
### App ###
########################################################################################################################

add_executable(StaticCalibration StaticCalibration.cpp ../src/utils/CommandLineParser.cpp ../include/StaticCalibration/utils/CommandLineParser.hpp)
target_link_libraries(StaticCalibration StaticCalibration-lib Eigen3::Eigen)
target_include_directories(StaticCalibration PUBLIC "${PROJECT_BINARY_DIR}")
#
#if (WITH_COVERAGE)
#    setup_target_for_coverage_gcovr_xml(
#            NAME StaticCalibration_coverage
#            EXECUTABLE StaticCalibration -j ${PROCESSOR_COUNT}
#    )
#endif ()