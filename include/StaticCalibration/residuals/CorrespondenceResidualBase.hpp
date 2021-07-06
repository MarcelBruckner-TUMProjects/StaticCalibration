//
// Created by brucknem on 11.04.21.
//

#ifndef CAMERASTABILIZATION_CORRESPONDENCERESIDUALBASE_HPP
#define CAMERASTABILIZATION_CORRESPONDENCERESIDUALBASE_HPP

#include "Eigen/Dense"
#include "ceres/ceres.h"
#include "StaticCalibration/objects/ParametricPoint.hpp"

namespace static_calibration {
    namespace calibration {
        namespace residuals {

            /**
             * A residual term used in the optimization process to find the camera pose.
             */
            class CorrespondenceResidualBase {
            protected:
                /**
                 * The expected [u, v] pixel corresponding to the world coordinate.
                 */
                Eigen::Matrix<double, 2, 1> expectedPixel;

                /**
                 * The parametricPoint that contains the correspondence.
                 */
                ParametricPoint parametricPoint;

            public:
                /**
                 * @constructor
                 *
                 * @param expectedPixel The expected [u, v] pixel location.
                 * @param point The [x, y, z] point that corresponds to the pixel.
                 * @param intrinsics The intrinsics of the pinhole camera model.
                 * @param imageSize The [width, height] of the image.
                 */
                CorrespondenceResidualBase(Eigen::Matrix<double, 2, 1> expectedPixel,
                                           const ParametricPoint &point);

                /**
                 * @destructor
                 */
                virtual ~CorrespondenceResidualBase() = default;
            };
        }
    }
}

#endif //CAMERASTABILIZATION_CORRESPONDENCERESIDUALBASE_HPP
