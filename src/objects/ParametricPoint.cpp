//
// Created by brucknem on 11.04.21.
//

#include <utility>
#include "StaticCalibration/objects/ParametricPoint.hpp"

namespace static_calibration {
    namespace calibration {
        ParametricPoint::ParametricPoint(Eigen::Matrix<double, 2, 1> expectedPixel, Eigen::Matrix<double, 3, 1> origin,
                                         const Eigen::Matrix<double, 3, 1>
                                         &axisA, double lambda, double lambdaMin, double lambdaMax) :
                expectedPixel(std::move(expectedPixel)), origin(std::move(origin)), axisA(axisA.stableNormalized()),
                lambda(new double(lambda)), lambdaMin(lambdaMin), lambdaMax(lambdaMax) {}

        Eigen::Matrix<double, 3, 1> ParametricPoint::getPosition() const {
            return origin + *lambda * axisA;
        }

        const Eigen::Matrix<double, 3, 1> &ParametricPoint::getOrigin() const {
            return origin;
        }

        const Eigen::Matrix<double, 3, 1> &ParametricPoint::getAxisA() const {
            return axisA;
        }

        double *ParametricPoint::getLambda() const {
            return lambda;
        }

        const Eigen::Matrix<double, 2, 1> &ParametricPoint::getExpectedPixel() const {
            return expectedPixel;
        }

        void ParametricPoint::setExpectedPixel(const Eigen::Matrix<double, 2, 1> &value) {
            expectedPixel = value;
        }

        ParametricPoint::ParametricPoint(Eigen::Matrix<double, 2, 1> expectedPixel, Eigen::Matrix<double, 3, 1> origin,
                                         const Eigen::Matrix<double, 3, 1> &axisA, double lambdaMax) :
                ParametricPoint(std::move(expectedPixel),
                                std::move(origin), axisA,
                                0, 0,
                                lambdaMax) {}

        ParametricPoint::ParametricPoint(Eigen::Matrix<double, 2, 1> expectedPixel, Eigen::Matrix<double, 3, 1> origin)
                : ParametricPoint(expectedPixel, origin, {0, 0, 0}, 0) {}

        double ParametricPoint::getLambdaMin() const {
            return lambdaMin;
        }

        double ParametricPoint::getLambdaMax() const {
            return lambdaMax;
        }
    }
}