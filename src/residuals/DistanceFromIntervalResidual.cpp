//
// Created by brucknem on 04.02.21.
//

#include "StaticCalibration/residuals/DistanceFromIntervalResidual.hpp"

namespace static_calibration {
	namespace calibration {
		namespace residuals {

			DistanceFromIntervalResidual::DistanceFromIntervalResidual(double upperBound) : DistanceFromIntervalResidual
																								(0, upperBound) {}

			DistanceFromIntervalResidual::DistanceFromIntervalResidual(double lowerBound, double upperBound)
				: lowerBound(
				lowerBound), upperBound(upperBound) {}

			template<typename T>
			bool DistanceFromIntervalResidual::operator()(const T *value, T *residual) const {
				if (value[0] > (T) upperBound) {
					residual[0] = value[0] - (T) upperBound;
				} else if (value[0] < (T) lowerBound) {
					residual[0] = value[0] - (T) lowerBound;
				} else {
					residual[0] = (T) 0;
				}
				return true;
			}

			ceres::CostFunction *DistanceFromIntervalResidual::create(double lowerBound, double upperBound) {
				return new ceres::AutoDiffCostFunction<DistanceFromIntervalResidual, 1, 1>(
					new DistanceFromIntervalResidual(lowerBound, upperBound)
				);
			}

			ceres::CostFunction *DistanceFromIntervalResidual::create(const double upperBound) {
				return new ceres::AutoDiffCostFunction<DistanceFromIntervalResidual, 1, 1>(
					new DistanceFromIntervalResidual(0, upperBound)
				);
			}

			template bool DistanceFromIntervalResidual::operator()(const ceres::Jet<double, 1> *, ceres::Jet<double, 1>
			*) const;

			template bool DistanceFromIntervalResidual::operator()(const double *, double *) const;
		}
	}
}
