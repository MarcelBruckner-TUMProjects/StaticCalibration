//
// Created by brucknem on 25.05.21.
//

#ifndef STATICCALIBRATION_FORMATTERS_HPP
#define STATICCALIBRATION_FORMATTERS_HPP

#include "StaticCalibration/CameraPoseEstimation.hpp"

namespace static_calibration {
    namespace utils {
        std::string toYAML(const static_calibration::calibration::CameraPoseEstimation &estimator);

        std::string toROSXML(const calibration::CameraPoseEstimation &estimator, const std::string &measurementPoint,
                             const std::string &cameraName);
    }
}


#endif //STATICCALIBRATION_FORMATTERS_HPP
