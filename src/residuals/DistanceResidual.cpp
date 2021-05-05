//
// Created by brucknem on 04.02.21.
//

#include "StaticCalibration/residuals/DistanceResidual.hpp"

namespace static_calibration {
	namespace calibration {
		namespace residuals {

			DistanceResidual::DistanceResidual(double expectedValue) : expectedValue(expectedValue) {}

			template<typename T>
			bool DistanceResidual::operator()(const T *value, T *residual) const {
				residual[0] = value[0] - (T) expectedValue;
				return true;
			}

			ceres::CostFunction *DistanceResidual::create(const double expectedValue) {
				return new ceres::AutoDiffCostFunction<DistanceResidual, 1, 1>(
					new DistanceResidual(expectedValue)
				);
			}

			template bool DistanceResidual::operator()(const ceres::Jet<double, 1> *, ceres::Jet<double, 1> *) const;

			template bool DistanceResidual::operator()(const double *, double *) const;
		}
	}
}
