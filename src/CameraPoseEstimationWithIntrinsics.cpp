//
// Created by brucknem on 06.07.21.
//

#include <StaticCalibration/residuals/CorrespondenceWithIntrinsicsResidual.hpp>
#include "StaticCalibration/CameraPoseEstimationWithIntrinsics.hpp"

namespace static_calibration {
    namespace calibration {

        void CameraPoseEstimationWithIntrinsics::evaluateIntrinsicsResiduals(ceres::Problem &problem) {
            intrinsicsLoss = evaluate(problem, intrinsicsResiduals);
        }

        ceres::Problem CameraPoseEstimationWithIntrinsics::solveProblem(bool logSummary) {
            auto problem = CameraPoseEstimationBase::solveProblem(logSummary);
            evaluateIntrinsicsResiduals(problem);
            return problem;
        }

        void CameraPoseEstimationWithIntrinsics::resetParameters() {
            intrinsics = initialIntrinsics;
            CameraPoseEstimationBase::resetParameters();
        }

        ceres::Problem CameraPoseEstimationWithIntrinsics::createProblem() {
            intrinsicsResiduals.clear();
            auto problem = CameraPoseEstimationBase::createProblem();
            addIntrinsicsConstraints(problem);

            return problem;
        }

        ceres::ResidualBlockId
        CameraPoseEstimationWithIntrinsics::addCorrespondenceResidualBlock(ceres::Problem &problem,
                                                                           const ParametricPoint &point,
                                                                           ceres::LossFunction *lossFunction) {
            return problem.AddResidualBlock(
                    residuals::CorrespondenceWithIntrinsicsResidual::create(
                            point.getExpectedPixel(),
                            point
                    ),
                    lossFunction,
                    &intrinsics[0],
                    &intrinsics[1],
                    &intrinsics[2],
                    &intrinsics[3],
                    &translation.x(),
                    &translation.y(),
                    &translation.z(),
                    &rotation.x(),
                    &rotation.y(),
                    &rotation.z(),
                    point.getLambda(),
                    weights[weights.size() - 1]
            );
        }

        void CameraPoseEstimationWithIntrinsics::addIntrinsicsConstraints(ceres::Problem &problem) {
            for (int i = 0; i < intrinsics.size(); ++i) {
                double lowerBound = std::max(500., initialIntrinsics[i] * 0.9);
                double upperBound = std::max(500., initialIntrinsics[i] * 1.1);
                double scale = getCorrespondenceLossUpperBound();

                intrinsicsResiduals.emplace_back(problem.AddResidualBlock(
                        static_calibration::calibration::residuals::DistanceFromIntervalResidual::create(
                                lowerBound, upperBound, "intrinsics"
                        ),
                        getScaledHuberLoss(scale),
                        &intrinsics[i]
                ));
            }
        }

        CameraPoseEstimationWithIntrinsics::CameraPoseEstimationWithIntrinsics(const std::vector<double> &intrinsics)
                : CameraPoseEstimationBase(intrinsics) {

        }

    }
}