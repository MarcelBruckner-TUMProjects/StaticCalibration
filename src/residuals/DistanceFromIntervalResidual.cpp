//
// Created by brucknem on 04.02.21.
//

#include "StaticCalibration/residuals/DistanceFromIntervalResidual.hpp"

#include <utility>

namespace static_calibration {
    namespace calibration {
        namespace residuals {

            DistanceFromIntervalResidual::DistanceFromIntervalResidual(double upperBound, std::string name)
                    : DistanceFromIntervalResidual
                              (0, upperBound, std::move(name)) {}

            DistanceFromIntervalResidual::DistanceFromIntervalResidual(double lowerBound, double upperBound,
                                                                       std::string name)
                    : lowerBound(
                    lowerBound), upperBound(upperBound), name(std::move(name)) {}

            template<typename T>
            bool DistanceFromIntervalResidual::operator()(const T *value, T *residual) const {
//                if (name == "intrinsics") {
//                    std::cout << name << std::endl;
//                }
                if (value[0] > (T) upperBound) {
                    residual[0] = value[0] - (T) upperBound;
                } else if (value[0] < (T) lowerBound) {
                    residual[0] = value[0] - (T) lowerBound;
                } else {
                    residual[0] = (T) 0;
                }
                return true;
            }

            ceres::CostFunction *
            DistanceFromIntervalResidual::create(double lowerBound, double upperBound, std::string name) {
                return new ceres::AutoDiffCostFunction<DistanceFromIntervalResidual, 1, 1>(
                        new DistanceFromIntervalResidual(lowerBound, upperBound, std::move(name))
                );
            }

            ceres::CostFunction *DistanceFromIntervalResidual::create(const double upperBound, std::string name) {
                return new ceres::AutoDiffCostFunction<DistanceFromIntervalResidual, 1, 1>(
                        new DistanceFromIntervalResidual(0, upperBound, std::move(name))
                );
            }

            template bool DistanceFromIntervalResidual::operator()(const ceres::Jet<double, 1> *, ceres::Jet<double, 1>
            *) const;

            template bool DistanceFromIntervalResidual::operator()(const double *, double *) const;
        }
    }
}
