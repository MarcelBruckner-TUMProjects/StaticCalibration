cmake_minimum_required(VERSION 3.10)

########################################################################################################################
### Packages ###
########################################################################################################################

find_package(Boost REQUIRED COMPONENTS program_options REQUIRED filesystem REQUIRED date_time)
include_directories(${Boost_INCLUDE_DIRS})

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(${PROJECT_SOURCE_DIR}/include)

########################################################################################################################
### Utils include ###
########################################################################################################################

add_library(AppUtils CommandLineParser.cpp)

target_include_directories(AppUtils
        INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
        PRIVATE ${CMAKE_CURRENT_BINARY_DIR}
        )
target_link_libraries(AppUtils ${Boost_LIBRARIES} Eigen3::Eigen)

########################################################################################################################
### App ###
########################################################################################################################

add_executable(StaticCalibration StaticCalibration.cpp CommandLineParser.cpp CommandLineParser.hpp)
target_link_libraries(StaticCalibration StaticCalibration-lib AppUtils Eigen3::Eigen)
target_include_directories(StaticCalibration PUBLIC "${PROJECT_BINARY_DIR}")
