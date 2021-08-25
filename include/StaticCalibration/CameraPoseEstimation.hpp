//
// Created by brucknem on 06.07.21.
//

#ifndef STATICCALIBRATION_CAMERAPOSEESTIMATION_HPP
#define STATICCALIBRATION_CAMERAPOSEESTIMATION_HPP

#include "CameraPoseEstimationBase.hpp"

namespace static_calibration {
    namespace calibration {
        class CameraPoseEstimation : public CameraPoseEstimationBase {
        protected:
            ceres::ResidualBlockId
            addCorrespondenceResidualBlock(ceres::Problem &problem, const ParametricPoint &point,
                                           ceres::LossFunction *lossFunction) override;

        public:
            explicit CameraPoseEstimation(const std::vector<double> &intrinsics);

            int getCorrespondenceLossUpperBound() const override;
        };
    }
}


#endif //STATICCALIBRATION_CAMERAPOSEESTIMATION_HPP
