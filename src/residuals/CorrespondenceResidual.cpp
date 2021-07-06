//
// Created by brucknem on 11.04.21.
//

#include "StaticCalibration/residuals/CorrespondenceResidual.hpp"

#include <utility>
#include "StaticCalibration/camera/RenderingPipeline.hpp"

namespace static_calibration {
    namespace calibration {
        namespace residuals {
            CorrespondenceResidual::CorrespondenceResidual(Eigen::Matrix<double, 2, 1> expectedPixel,
                                                           const ParametricPoint &point, std::vector<double> intrinsics)
                    : CorrespondenceResidualBase(std::move(expectedPixel), point), intrinsics(std::move(intrinsics)) {
                if (intrinsics.size() == 4) {
                    intrinsics.emplace_back(1);
                }
            }

            template<typename T>
            bool CorrespondenceResidual::operator()(
                    const T *tx,
                    const T *ty,
                    const T *tz,
                    const T *rx,
                    const T *ry,
                    const T *rz,
                    const T *lambda,
                    const T *mu,
                    const T *weight,
                    T *residual) const {
                Eigen::Matrix<T, 3, 1> point = parametricPoint.getOrigin().cast<T>();
                point += parametricPoint.getAxisA().cast<T>() * lambda[0];
                point += parametricPoint.getAxisB().cast<T>() * mu[0];

                Eigen::Matrix<T, 2, 1> actualPixel;
                bool flipped;
                actualPixel = static_calibration::camera::render(
                        new T[3]{tx[0], ty[0], tz[0]},
                        new T[3]{rx[0], ry[0], rz[0]},
                        new T[5]{(T) intrinsics[0], (T) intrinsics[1], (T) intrinsics[2], (T) intrinsics[3],
                                 (T) intrinsics[4]},
                        point.data(),
                        flipped
                );

                residual[0] = expectedPixel.x() - actualPixel.x();
                residual[1] = expectedPixel.y() - actualPixel.y();

                residual[0] = residual[0] * weight[0];
                residual[1] = residual[1] * weight[0];

                return !flipped;
            }

            ceres::CostFunction *
            CorrespondenceResidual::create(const Eigen::Matrix<double, 2, 1> &expectedPixel,
                                           const ParametricPoint &point, const std::vector<double> &intrinsics) {
                return new ceres::AutoDiffCostFunction<CorrespondenceResidual, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1>(
                        new CorrespondenceResidual(expectedPixel, point, intrinsics),
                        ceres::TAKE_OWNERSHIP
                );
            }
        }
    }
}