//
// Created by brucknem on 11.04.21.
//

#include <utility>
#include "StaticCalibration/objects/ParametricPoint.hpp"

namespace static_calibration {
	namespace calibration {
		ParametricPoint::ParametricPoint(Eigen::Matrix<double, 3, 1> origin, const Eigen::Matrix<double, 3, 1>
		&axisA, const Eigen::Matrix<double, 3, 1> &axisB, double lambda, double mu) :
			expectedPixel({0, 0}), origin(std::move(origin)), axisA(axisA.normalized()),
			axisB(axisB.normalized()),
			lambda(new double(lambda)), mu(new double(mu)) {}

		Eigen::Matrix<double, 3, 1> ParametricPoint::getPosition() const {
			return origin + *lambda * axisA + *mu * axisB;
		}

		const Eigen::Matrix<double, 3, 1> &ParametricPoint::getOrigin() const {
			return origin;
		}

		const Eigen::Matrix<double, 3, 1> &ParametricPoint::getAxisA() const {
			return axisA;
		}

		const Eigen::Matrix<double, 3, 1> &ParametricPoint::getAxisB() const {
			return axisB;
		}

		double *ParametricPoint::getLambda() const {
			return lambda;
		}

		double *ParametricPoint::getMu() const {
			return mu;
		}

		const Eigen::Matrix<double, 2, 1> &ParametricPoint::getExpectedPixel() const {
			return expectedPixel;
		}

		void ParametricPoint::setExpectedPixel(const Eigen::Matrix<double, 2, 1> &value) {
			expectedPixel = value;
			isExpectedPixelSet = true;
		}

		ParametricPoint
		ParametricPoint::onPlane(const Eigen::Matrix<double, 2, 1> &expectedPixel, Eigen::Matrix<double, 3, 1> origin,
								 const Eigen::Matrix<double, 3, 1> &axisA, const Eigen::Matrix<double, 3, 1> &axisB,
								 double lambda, double mu) {
			ParametricPoint point = onPlane(std::move(origin), axisA, axisB, lambda, mu);
			point.setExpectedPixel(expectedPixel);
			return point;
		}

		ParametricPoint
		ParametricPoint::onPlane(Eigen::Matrix<double, 3, 1> origin, const Eigen::Matrix<double, 3, 1> &axisA,
								 const Eigen::Matrix<double, 3, 1> &axisB, double lambda, double mu) {
			return ParametricPoint(std::move(origin), axisA.stableNormalized(), axisB.stableNormalized(), lambda,
								   mu);
		}

		ParametricPoint
		ParametricPoint::onLine(const Eigen::Matrix<double, 2, 1> &expectedPixel, Eigen::Matrix<double, 3, 1> origin,
								const
								Eigen::Matrix<double, 3, 1> &heading, double lambda) {
			ParametricPoint point = onLine(std::move(origin), heading, lambda);
			point.setExpectedPixel(expectedPixel);
			return point;
		}

		ParametricPoint
		ParametricPoint::onLine(Eigen::Matrix<double, 3, 1> origin, const Eigen::Matrix<double, 3, 1> &heading,
								double lambda) {
			return ParametricPoint::onPlane(std::move(origin), heading, {0, 0, 0}, lambda, 0);
		}

		ParametricPoint
		ParametricPoint::onPoint(const Eigen::Matrix<double, 2, 1> &expectedPixel,
								 const Eigen::Matrix<double, 3, 1> &worldPosition) {
			ParametricPoint point = onPoint(worldPosition);
			point.setExpectedPixel(expectedPixel);
			return point;
		}

		ParametricPoint ParametricPoint::onPoint(const Eigen::Matrix<double, 3, 1> &worldPosition) {
			return ParametricPoint::onLine(worldPosition, {0, 0, 0}, 0);
		}

		bool ParametricPoint::hasExpectedPixel() const {
			return isExpectedPixelSet;
		}

		ParametricPoint::ParametricPoint(ParametricPoint &other,
										 const Eigen::Matrix<double, 2, 1> &expectedPixel) : ParametricPoint
																								 (
																									 other.origin,
																									 other.axisA,
																									 other.axisB,
																									 0, 0
																								 ) {
			setExpectedPixel(expectedPixel);
		}
	}
}