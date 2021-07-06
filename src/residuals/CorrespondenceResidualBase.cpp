//
// Created by brucknem on 11.04.21.
//

#include "StaticCalibration/residuals/CorrespondenceResidualBase.hpp"

namespace static_calibration {
    namespace calibration {
        namespace residuals {
            CorrespondenceResidualBase::CorrespondenceResidualBase(
                    Eigen::Matrix<double, 2, 1> expectedPixel,
                    const ParametricPoint &point) :
                    expectedPixel(std::move(expectedPixel)),
                    parametricPoint(point) {}
        }
    }
}