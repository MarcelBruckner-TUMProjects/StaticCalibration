//
// Created by brucknem on 04.02.21.
//

#ifndef CAMERASTABILIZATION_RENDERINGPIPELINE_HPP
#define CAMERASTABILIZATION_RENDERINGPIPELINE_HPP

#include "Eigen/Dense"
#include <vector>

#include "CMakeConfig.h"

#ifdef WITH_OPENCV

#include "opencv2/opencv.hpp"

#endif //WITH_OPENCV

namespace static_calibration {
    namespace camera {

        /**
         * Creates the camera intrinsic matrix.
         * https://en.wikipedia.org/wiki/Camera_resectioning
         *
         * @tparam T double or ceres::Jet
         * @param intrinsics [focalLength, sensorWidth, sensorHeight, principalX, principalY, skew]
         *
         * @return The intrinsics camera matrix.
         */
        template<typename T>
        std::vector<T> getIntrinsicsFromRealSensor(const T *intrinsics);

        template<typename T>
        Eigen::Matrix<T, 3, 4> getIntrinsicsMatrix(const T *intrinsics);

        template<typename T>
        Eigen::Matrix<T, 3, 4> getIntrinsicsMatrix(const T *intrinsics, bool &invalid);

        /**
         * Creates the camera intrinsic matrix.
         * https://en.wikipedia.org/wiki/Camera_resectioning
         *
         * @tparam T double or ceres::Jet
         * @param intrinsics [alphaX, skew, u0, 0, alphaY, v0, 0, 0, 1]
         *
         * @return The intrinsics camera matrix.
         */
        template<typename T>
        Eigen::Matrix<T, 3, 4> getIntrinsicsMatrixFromConfig(const T *intrinsics);

        /**
         * Creates the camera intrinsic matrix for the mock blender setup.
         *
         * @return The intrinsics camera matrix.
         */
        std::vector<double> getBlenderCameraIntrinsics();

        /**
         * Creates the camera intrinsic matrix for the s40 n cam far camera.
         *
         * @return The intrinsics camera matrix.
         */
        std::vector<double> getS40NCamFarIntrinsics();

        /**
         * Divides a pixel in homogeneous coordinates by its z component.
         *
         * @tparam T double or ceres::Jet
         * @param vector The (possibly) unnormalized [x, y, z] vector.
         *
         * @return The normalized [x/z, y/z] vector.
         */
        template<typename T>
        Eigen::Matrix<T, 2, 1> perspectiveDivision(const Eigen::Matrix<T, 3, 1> &vector, bool &flipped);

        /**
         * Generates a camera rotation matrix from the euler angle representation.<br>
         * Internally creates the matrices for a rotation around the X/Y/Z axis and concatenates them.<br>
         * Assumes a camera with no rotation to have a coordinate system of [X/Y/-Z].<br>
         *
         * @link https://en.wikipedia.org/wiki/Rotation_matrix#In_three_dimensions
         *
         * @tparam T double or ceres::Jet
         * @param rotation The [x, y, z] euler angle rotation.
         *
         * @return The complete rotation around the three axis.
         */
        template<typename T>
        Eigen::Matrix<T, 4, 4> getCameraRotationMatrix(const T *rotation);

        /**
         * Transforms the given vector from world to camera space.
         *
         * @tparam T double or ceres::Jet
         * @param translation The [x, y, z] translation of the camera in world space.
         * @param rotation The [x, y, z] euler angle rotation of the camera around the world axis.
         * @param vector The [x, y, z, w] vector in world space.
         *
         * @return The [x, y, z, w] vector in camera space.
         */
        template<typename T>
        Eigen::Matrix<T, 4, 1> toCameraSpace(const T *translation, const T *rotation, const T *vector);

        /**
         * Wrapper for the whole rendering pipeline.
         *
         * @tparam T double or ceres::Jet
         * @param translation The [x, y, z] translation of the camera in world space.
         * @param rotation The [x, y, z] euler angle rotation of the camera around the world axis.
         * @param intrinsics [focalLength, sensorWidth, sensorHeight, principalX, principalY, skew]
         * @param vector The [x, y, z, w] vector in world space.
         *
         * @return The [u, v] expectedPixel location in image space.
         */
        template<typename T>
        Eigen::Matrix<T, 2, 1>
        render(const T *translation, const T *rotation, const T *intrinsics, const T *vector);

        template<typename T>
        Eigen::Matrix<T, 2, 1>
        render(const T *translation, const T *rotation, const T *intrinsics, const T *vector, bool &flipped);

    }
}
#endif //CAMERASTABILIZATION_RENDERINGPIPELINE_HPP
