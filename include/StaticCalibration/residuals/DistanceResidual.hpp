//
// Created by brucknem on 04.02.21.
//

#ifndef CAMERASTABILIZATION_DISTANCERESIDUAL_HPP
#define CAMERASTABILIZATION_DISTANCERESIDUAL_HPP

#include "Eigen/Dense"
#include "ceres/ceres.h"

namespace static_calibration {
	namespace calibration {
		namespace residuals {

			/**
			 * Residual for the distance of a value to a given fixed value.
			 * Minimizes f(x, e) = |e - x|
			 */
			class DistanceResidual {
			protected:

				/**
				 * The expected value.
				 */
				double expectedValue;

			public:
				/**
				 * @constructor
				 *
				 * @param expectedValue The expected value to calculate the distance to.
				 */
				explicit DistanceResidual(double expectedValue);

				/**
				 * @destructor
				 */
				virtual ~DistanceResidual() = default;

				/**
				 * Residual calculation function.
				 *
				 * @tparam T double or ceres::Jet<double, 1>
				 *
				 * @param value The current estimated value.
				 * @param residual The residual, i.e. |expected - value|
				 *
				 * @return true
				 */
				template<typename T>
				bool operator()(const T *value, T *residual) const;

				/**
				 * Factory method to ease residual creation.
				 *
				 * @param expectedValue The expected value to calculate the distance to.
				 *
				 * @return The cost function based on the residual.
				 */
				static ceres::CostFunction *create(double expectedValue);

			};
		}
	}
}

#endif //CAMERASTABILIZATION_DISTANCERESIDUAL_HPP
