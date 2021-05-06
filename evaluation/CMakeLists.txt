cmake_minimum_required(VERSION 3.10)

########################################################################################################################
### Packages ###
########################################################################################################################

if (WITH_OPENCV)
    find_package(OpenCV REQUIRED)
    include_directories(${OpenCV_INCLUDE_DIRS})

    list(APPEND EXTRA_LIBS ${OpenCV_LIBS})
endif ()

find_package(Boost REQUIRED COMPONENTS program_options REQUIRED filesystem REQUIRED date_time)
include_directories(${Boost_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/include)

########################################################################################################################
### Utils include ###
########################################################################################################################

add_library(Utils CSVWriter.cpp Commons.cpp)

target_include_directories(Utils
        INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
        PRIVATE ${CMAKE_CURRENT_BINARY_DIR}
        )

target_link_libraries(Utils
        ${Boost_LIBRARIES}
        StaticCalibration-lib
        ${EXTRA_LIBS})


########################################################################################################################
### Visualize ###
########################################################################################################################

########################################################################################################################
### Evaluation ###
########################################################################################################################

add_executable(EvaluateStaticCalibration EvaluateStaticCalibration.cpp)
target_link_libraries(EvaluateStaticCalibration StaticCalibration-lib Utils ${OpenCV_LIBS} ${Boost_LIBRARIES})
target_include_directories(EvaluateStaticCalibration PUBLIC "${PROJECT_BINARY_DIR}")
