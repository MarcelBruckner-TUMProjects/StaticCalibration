//
// Created by brucknem on 25.05.21.
//

#ifndef STATICCALIBRATION_FORMATTERS_HPP
#define STATICCALIBRATION_FORMATTERS_HPP

#include "StaticCalibration/CameraPoseEstimationBase.hpp"
#include "StaticCalibration/CameraPoseEstimationWithIntrinsics.hpp"

namespace static_calibration {
    namespace utils {
        std::string toYAML(const calibration::CameraPoseEstimationWithIntrinsics &estimator);

        std::string toYAML(const static_calibration::calibration::CameraPoseEstimationBase &estimator);

        std::string
        toROStf2Node(const calibration::CameraPoseEstimationBase &estimator, const std::string &measurementPoint,
                     const std::string &cameraName);

        std::string
        toROSParamsIntrinsics(const calibration::CameraPoseEstimationWithIntrinsics &estimator,
                              const std::string &measurementPoint,
                              const std::string &cameraName);
    }
}


#endif //STATICCALIBRATION_FORMATTERS_HPP
