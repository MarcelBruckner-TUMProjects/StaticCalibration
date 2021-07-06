//
// Created by brucknem on 11.04.21.
//

#ifndef CAMERASTABILIZATION_CORRESPONDENCEWITHINTRINSICSRESIDUAL_HPP
#define CAMERASTABILIZATION_CORRESPONDENCEWITHINTRINSICSRESIDUAL_HPP

#include "Eigen/Dense"
#include "ceres/ceres.h"
#include "StaticCalibration/objects/ParametricPoint.hpp"
#include "StaticCalibration/residuals/CorrespondenceResidualBase.hpp"

namespace static_calibration {
    namespace calibration {
        namespace residuals {

            /**
             * A residual term used in the optimization process to find the camera pose.
             */
            class CorrespondenceWithIntrinsicsResidual : public CorrespondenceResidualBase {

            public:
                /**
                 * @constructor
                 *
                 * @param expectedPixel The expected [u, v] pixel location.
                 * @param point The [x, y, z] point that corresponds to the pixel.
                 * @param intrinsics The intrinsics of the pinhole camera model.
                 * @param imageSize The [width, height] of the image.
                 */
                CorrespondenceWithIntrinsicsResidual(Eigen::Matrix<double, 2, 1> expectedPixel,
                                                     const ParametricPoint &point);

                /**
                 * @destructor
                 */
                virtual ~CorrespondenceWithIntrinsicsResidual() = default;

                /**
                 * Calculates the residual error after transforming the world position to a pixel.
                 *
                 * @tparam T Template parameter expected from the ceres-solver.
                 * @param translation The [x, y, z] translation of the camera in world space for which we optimize.
                 * @param rotation The [x, y, z] euler angle rotation of the camera around the world axis for which we optimize.
                 * @param lambda The [l] distance of the point in the direction of one side of the parametricPoint from the origin.
                 * @param mu The [m] distance of the point in the direction of another side of the parametricPoint from the origin.
                 * @param residual The [u, v] pixel error between the expected and calculated pixel.
                 * @return true
                 */
                template<typename T>
                bool operator()(
                        const T *f_x,
                        const T *f_y,
                        const T *fy,
                        const T *cy,
                        const T *tx,
                        const T *ty,
                        const T *tz,
                        const T *rx,
                        const T *ry,
                        const T *rz,
                        const T *lambda,
                        const T *mu,
                        const T *weight,
                        T *residual)
                const;

                /**
                 * Factory method to hide the residual creation.
                 */
                static ceres::CostFunction *create(const Eigen::Matrix<double, 2, 1> &expectedPixel,
                                                   const ParametricPoint &point);
            };
        }
    }
}

#endif //CAMERASTABILIZATION_CORRESPONDENCEWITHINTRINSICSRESIDUAL_HPP
