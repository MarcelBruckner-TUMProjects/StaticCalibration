########################################################################################################################
### Includes ###
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

include_directories(${PROJECT_SOURCE_DIR}/extern)
include_directories(${PROJECT_SOURCE_DIR}/include)

########################################################################################################################
### Library ###
########################################################################################################################

add_library(StaticCalibration-lib
        CameraPoseEstimationBase.cpp
        CameraPoseEstimation.cpp
        CameraPoseEstimationWithIntrinsics.cpp

        camera/RenderingPipeline.cpp

        residuals/DistanceFromIntervalResidual.cpp
        residuals/DistanceResidual.cpp
        residuals/CorrespondenceResidualBase.cpp
        residuals/CorrespondenceResidual.cpp
        residuals/CorrespondenceWithIntrinsicsResidual.cpp

        objects/ParametricPoint.cpp
        objects/WorldObject.cpp
        objects/YAMLExtension.cpp

        utils/CommandLineParser.cpp
        utils/Formatters.cpp
        utils/RenderUtils.cpp
        objects/ImageObject.cpp
        objects/DataSet.cpp)

target_include_directories(StaticCalibration-lib
        INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
        PRIVATE ${CMAKE_CURRENT_BINARY_DIR}
        PUBLIC ${PROJECT_BINARY_DIR}
        )

target_link_libraries(StaticCalibration-lib
        Eigen3::Eigen
        Ceres::ceres
        yaml-cpp
        tinyxml2
        ${Boost_LIBRARIES}
        ${EXTRA_LIBS}
        )
