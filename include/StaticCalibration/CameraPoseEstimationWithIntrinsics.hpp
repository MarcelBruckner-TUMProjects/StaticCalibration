//
// Created by brucknem on 06.07.21.
//

#ifndef STATICCALIBRATION_CAMERAPOSEESTIMATIONWITHINTRINSICS_HPP
#define STATICCALIBRATION_CAMERAPOSEESTIMATIONWITHINTRINSICS_HPP


#include <utility>

#include "CameraPoseEstimationBase.hpp"

namespace static_calibration {
    namespace calibration {

        class CameraPoseEstimationWithIntrinsics : public CameraPoseEstimationBase {
        protected:

            /**
             * The ids of the intrinsic parameter residual blocks.
             */
            std::vector<ceres::ResidualBlockId> intrinsicsResiduals;


            /**
             * Evaluates the problem for the intrinsics residuals.
             *
             * @param problem The ceres problem.
             *
             * @return The loss of the residuals.
             */
            void evaluateIntrinsicsResiduals(ceres::Problem &problem);

        public:

            explicit CameraPoseEstimationWithIntrinsics(const std::vector<double> &intrinsics);

            ~CameraPoseEstimationWithIntrinsics() override = default;


            void addIntrinsicsConstraints(ceres::Problem &problem);

            ceres::Problem solveProblem(bool logSummary) override;

            void resetParameters() override;

            ceres::Problem createProblem() override;

            ceres::ResidualBlockId
            addCorrespondenceResidualBlock(ceres::Problem &problem, const ParametricPoint &point,
                                           ceres::LossFunction *lossFunction) override;

        };
    }
}

#endif //STATICCALIBRATION_CAMERAPOSEESTIMATIONWITHINTRINSICS_HPP
