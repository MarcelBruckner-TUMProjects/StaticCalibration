//
// Created by brucknem on 11.04.21.
//

#ifndef CAMERASTABILIZATION_PARAMETRICPOINT_HPP
#define CAMERASTABILIZATION_PARAMETRICPOINT_HPP

#include <Eigen/Dense>

namespace static_calibration {
    namespace calibration {
        /**
         * A point lying in a 2-dimensional parametricPoint defined by an origin point lying in the parametricPoint and two axis.
         */
        class ParametricPoint {
        protected:
            /**
             * The origin of the parametricPoint. This point lies within the parametricPoint.
             */
            Eigen::Matrix<double, 3, 1> origin;

            /**
             * A normalized axis of the parametricPoint.
             */
            Eigen::Matrix<double, 3, 1> axisA;

            /**
             * The expected pixel location of the point.
             */
            Eigen::Matrix<double, 2, 1> expectedPixel;

            /**
             * The distance from the origin in the first axis.
             */
            double *lambda;

            double lambdaMin;

            double lambdaMax;

        public:
            /**
             * @constructor
             *
             * @param origin The origin of the parametricPoint.
             * @param axisA One side of the parametricPoint.
             * @param lambda Optional distance from the origin in the first axis.
             */
            ParametricPoint(Eigen::Matrix<double, 2, 1> expectedPixel, Eigen::Matrix<double, 3, 1> origin,
                            const Eigen::Matrix<double, 3, 1> &axisA,
                            double lambda, double lambdaMin, double lambdaMax);

            ParametricPoint(Eigen::Matrix<double, 2, 1> expectedPixel, Eigen::Matrix<double, 3, 1> origin,
                            const Eigen::Matrix<double, 3, 1> &axisA, double lambdaMax);

            ParametricPoint(Eigen::Matrix<double, 2, 1> expectedPixel, Eigen::Matrix<double, 3, 1> origin);

            /**
             * @destructor
             */
            virtual ~ParametricPoint() = default;

            /**
             * @get The world position of the point.
             */
            Eigen::Matrix<double, 3, 1> getPosition() const;

            /**
             * @get The world position of the origin.
             */
            const Eigen::Matrix<double, 3, 1> &getOrigin() const;

            /**
             * @get The world direction of the first axis.
             */
            const Eigen::Matrix<double, 3, 1> &getAxisA() const;

            /**
             * @get The length of the first axis.
             */
            double *getLambda() const;

            /**
             * @get
             */
            const Eigen::Matrix<double, 2, 1> &getExpectedPixel() const;

            /**
             * @set
             */
            void setExpectedPixel(const Eigen::Matrix<double, 2, 1> &expectedPixel);

            double getLambdaMin() const;

            double getLambdaMax() const;
        };
    }
}

#endif //CAMERASTABILIZATION_PARAMETRICPOINT_HPP
