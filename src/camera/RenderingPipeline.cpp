//
// Created by brucknem on 04.02.21.
//

#include "StaticCalibration/camera/RenderingPipeline.hpp"
#include "ceres/ceres.h"
#include "CMakeConfig.h"

namespace static_calibration {
    namespace camera {

        template<typename T>
        Eigen::Matrix<T, 2, 1>
        render(const T *translation, const T *rotation, const T *intrinsics, const T *vector) {
            bool flipped;
            return render(translation, rotation, intrinsics, vector, flipped);
        }

        template Eigen::Matrix<double, 2, 1>
        render(const double *translation, const double *rotation, const double *intrinsics, const double *vector);

        template Eigen::Matrix<ceres::Jet<double, 14>, 2, 1>
        render(const ceres::Jet<double, 14> *translation, const ceres::Jet<double, 14> *rotation,
               const ceres::Jet<double, 14> *intrinsics, const ceres::Jet<double, 14> *vector);

        template<typename T>
        Eigen::Matrix<T, 2, 1>
        render(const T *translation, const T *rotation, const T *intrinsics, const T
        *vector, bool &flipped) {
//			Eigen::Matrix<T, 3, 1> translationVector{translation[0], translation[1], translation[2]};
//			Eigen::Matrix<T, 3, 1> rotationVector{rotation[0], rotation[1], rotation[2]};
//			Eigen::Matrix<T, 3, 1> vectorVector{vector[0], vector[1], vector[2]};

//			std::cout << "Translation" << std::endl << translationVector << std::endl;
//			std::cout << "Rotation" << std::endl << rotationVector << std::endl;
//			std::cout << "Vector" << std::endl << vectorVector << std::endl;

            Eigen::Matrix<T, 4, 1> pointInCameraSpace = toCameraSpace(translation, rotation, vector);
//			std::cout << "Camera Space" << std::endl << pointInCameraSpace << std::endl;

            bool invalid;
            auto intrinsicsMatrix = getIntrinsicsMatrix(intrinsics, invalid);
//			std::cout << intrinsicsMatrix << std::endl;

            Eigen::Matrix<T, 3, 1> homogeneousPixel = intrinsicsMatrix * pointInCameraSpace;
//			std::cout << "Homogeneous pixel" << std::endl << homogeneousPixel << std::endl;

            bool internalFlipped;
            Eigen::Matrix<T, 2, 1> pixel = perspectiveDivision(homogeneousPixel, internalFlipped);
//			std::cout << "Pixel" << std::endl << pixel << std::endl;
            flipped = internalFlipped || invalid;
            return pixel;
        }

        template Eigen::Matrix<double, 2, 1>
        render<double>(const double *, const double *, const double *, const double *, bool &);

        template Eigen::Matrix<ceres::Jet<double, 14>, 2, 1>
        render<ceres::Jet<double, 14>>(const ceres::Jet<double, 14> *, const ceres::Jet<double, 14> *,
                                       const ceres::Jet<double, 14> *, const ceres::Jet<double, 14> *, bool &);

        template<typename T>
        Eigen::Matrix<T, 4, 4> getCameraRotationMatrix(const T *rotation) {
            Eigen::Matrix<T, 3, 1> rotationInRadians{-rotation[0], -rotation[1], rotation[2]};
            rotationInRadians *= (T) (M_PI / 180);

            T zero = (T) 0;
            T one = (T) 1;

            Eigen::Matrix<T, 4, 4> zAxis;
            T theta = rotationInRadians.z();
            zAxis << cos(theta), -sin(theta), zero, zero,
                    sin(theta), cos(theta), zero, zero,
                    zero, zero, one, zero,
                    zero, zero, zero, one;

            Eigen::Matrix<T, 4, 4> yAxis;
            theta = rotationInRadians.y();
            yAxis << cos(theta), zero, sin(theta), zero,
                    zero, one, zero, zero,
                    -sin(theta), zero, cos(theta), zero,
                    zero, zero, zero, one;

            Eigen::Matrix<T, 4, 4> xAxis;
            theta = rotationInRadians.x();
            xAxis << one, zero, zero, zero,
                    zero, cos(theta), -sin(theta), zero,
                    zero, sin(theta), cos(theta), zero,
                    zero, zero, zero, one;

            Eigen::Matrix<T, 4, 4> identityZFlipped;
            identityZFlipped << one, zero, zero, zero,
                    zero, one, zero, zero,
                    zero, zero, -one, zero,
                    zero, zero, zero, one;

            const Eigen::Matrix<T, 4, 4> &rotationMatrix = identityZFlipped * zAxis * yAxis * xAxis;
            return rotationMatrix;
        }

        template<typename T>
        Eigen::Matrix<T, 2, 1> perspectiveDivision(const Eigen::Matrix<T, 3, 1> &vector, bool &flipped) {
            flipped = vector[2] < (T) 0;
            Eigen::Matrix<T, 3, 1> result = vector / (vector[2] + 1e-51);
            return Eigen::Matrix<T, 2, 1>{result(0, 0), result(1, 0)};
        }

        template<typename T>
        Eigen::Matrix<T, 4, 1> toCameraSpace(const T *translation, const T *rotation, const T *vector) {
            // TODO Remove inverse
            return getCameraRotationMatrix<T>(rotation).inverse() *
                   (Eigen::Matrix<T, 4, 1>(
                           vector[0] - translation[0],
                           vector[1] - translation[1],
                           vector[2] - translation[2],
                           (T) 1));
        }

#ifdef WITH_OPENCV
        Eigen::Matrix<double, 2, 1> render(const Eigen::Vector3d &translation, const Eigen::Vector3d &rotation,
                                               const std::vector<double> &intrinsics,
                                               const Eigen::Vector4d &vector,
                                               const cv::Vec3d &color, cv::Mat &image) {
                bool flipped;
                return render(translation, rotation, intrinsics, vector, color, image, flipped);
            }

            Eigen::Matrix<double, 2, 1> render(const Eigen::Vector3d &translation, const Eigen::Vector3d &rotation,
                                               const std::vector<double> &intrinsics,
                                               const Eigen::Vector4d &vector,
                                               const cv::Vec3d &color, cv::Mat &image, bool &flipped) {
                Eigen::Vector2i imageSize(image.cols, image.rows);

                Eigen::Vector2d pointInImageSpace = render(
                    translation.data(), rotation.data(), intrinsics.data(),
                    vector.data(), flipped);

                if (flipped) {
                    return pointInImageSpace;
                }
                int imageHeight = imageSize.y() - 1;

                for (int i = 0; i < 2; ++i) {
                    for (int j = 0; j < 2; ++j) {
                        Eigen::Vector2i nearestPixel = pointInImageSpace.cast<int>();
                        nearestPixel.x() += i;
                        nearestPixel.y() += j;
                        double distance = (nearestPixel.cast<double>() - pointInImageSpace).norm();
                        nearestPixel.y() = imageHeight - nearestPixel.y();

    //					std::cout << "[" << nearestPixel.x() << ", " << nearestPixel.y() << "] - " << distance;
                        if (nearestPixel.x() >= imageSize.x() || nearestPixel.y() >= imageSize.y() ||
                            nearestPixel.x() < 0 || nearestPixel.y() < 0) {
                            continue;
                        }
    //					std::cout << std::endl;

                        cv::Vec4d colorCV = {color[0], color[1], color[2], (distance / (double) sqrt(2))};

                        // 1200, 1920
                        image.at<cv::Vec4d>(nearestPixel.y(), nearestPixel.x()) = colorCV;
                    }
                }

                cv::circle(image,
                           {(int) pointInImageSpace.x(), (int) (imageHeight - pointInImageSpace.y())},
                           std::min(5, std::max(0, (int) (imageSize.y() * 0.01))),
                           color,
                           cv::FILLED
                );
                return pointInImageSpace;
            }
#endif //WITH_OPENCV

        template<typename T>
        Eigen::Matrix<T, 3, 4> getIntrinsicsMatrix(const T *intrinsics) {
            bool invalid;
            return getIntrinsicsMatrix(intrinsics, invalid);
        }

        template Eigen::Matrix<double, 3, 4> getIntrinsicsMatrix(const double *intrinsics);

        template Eigen::Matrix<ceres::Jet<double, 14>, 3, 4>
        getIntrinsicsMatrix(const ceres::Jet<double, 14> *intrinsics);

        template<typename T>
        Eigen::Matrix<T, 3, 4> getIntrinsicsMatrix(const T *intrinsics, bool &invalid) {
            T zero = (T) 0;
            invalid = intrinsics[0] < zero;
            invalid = false;
            return getIntrinsicsMatrixFromConfig(new T[9]{
                    intrinsics[0], zero, intrinsics[2],
                    zero, intrinsics[0] * intrinsics[1], intrinsics[3],
//				zero, intrinsics[4], (T) 1
                    zero, zero, (T) 1
            });
        }

        template Eigen::Matrix<double, 3, 4> getIntrinsicsMatrix(const double *intrinsics, bool &invalid);

        template Eigen::Matrix<ceres::Jet<double, 14>, 3, 4>
        getIntrinsicsMatrix(const ceres::Jet<double, 14> *intrinsics, bool &invalid);

        template<typename T>
        std::vector<T> getIntrinsicsFromRealSensor(const T *intrinsics) {
            T zero = (T) 0;
            T focalLength = intrinsics[0];

            Eigen::Matrix<T, 2, 1> principalPoint = Eigen::Matrix<T, 2, 1>(
                    intrinsics[3],
                    intrinsics[4]
            );

            T skew = intrinsics[5];

            T alpha = focalLength * (T) 1. / intrinsics[1];

            return
                    std::vector<T>
                            {
                                    alpha, intrinsics[2], principalPoint(0, 0), principalPoint(1, 0), skew
                            };
        }

        template std::vector<double> getIntrinsicsFromRealSensor(const double *intrinsics);

        template std::vector<ceres::Jet<double, 14>>
        getIntrinsicsFromRealSensor(const ceres::Jet<double, 14> *intrinsics);

        template<typename T>
        Eigen::Matrix<T, 3, 4> getIntrinsicsMatrixFromConfig(const T *intrinsics) {
            Eigen::Matrix<T, 3, 4> matrix;
            T zero = T(0);
            matrix <<
                   intrinsics[0], intrinsics[1], intrinsics[2], zero,
                    intrinsics[3], intrinsics[4], intrinsics[5], zero,
                    intrinsics[6], intrinsics[7], intrinsics[8], zero;
//			std::cout << matrix << std::endl;
            return matrix;
        }

        template Eigen::Matrix<double, 3, 4> getIntrinsicsMatrixFromConfig(const double *intrinsics);

        template Eigen::Matrix<ceres::Jet<double, 14>, 3, 4>
        getIntrinsicsMatrixFromConfig(const ceres::Jet<double, 14> *intrinsics);

        std::vector<double> getBlenderCameraIntrinsics() {
            double pixelWidth = 32. / 1920.;
            double principalX = 1920. / 2;
            double principalY = 1200. / 2;
            std::vector<double> intrinsics{
                    20, pixelWidth, 1, principalX, principalY, 0
            };
            return getIntrinsicsFromRealSensor(intrinsics.data());
        }

        std::vector<double> getS40NCamFarIntrinsics() {
            return std::vector<double>{
                    9023.482825, 1., 1222.314303, 557.541182, 0.000000
            };
        }

    }
}