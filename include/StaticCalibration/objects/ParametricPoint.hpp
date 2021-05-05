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
			 * Another normalized axis of the parametricPoint.
			 */
			Eigen::Matrix<double, 3, 1> axisB;

			/**
			 * The expected pixel location of the point.
			 */
			Eigen::Matrix<double, 2, 1> expectedPixel;

			/**
			 * The distance from the origin in the first axis.
			 */
			double *lambda;

			/**
			 * The distance from the origin in the second axis.
			 */
			double *mu;

			/**
			 * Flag if the expected pixel is set.
			 */
			bool isExpectedPixelSet = false;

		public:
			/**
			 * @constructor
			 *
			 * @param origin The origin of the parametricPoint.
			 * @param axisA One side of the parametricPoint.
			 * @param axisB Another side of the parametricPoint.
			 * @param lambda Optional distance from the origin in the first axis.
			 * @param mu Optional distance from the origin in the second axis.
			 */
			ParametricPoint(Eigen::Matrix<double, 3, 1> origin, const Eigen::Matrix<double, 3, 1> &axisA,
							const Eigen::Matrix<double, 3, 1> &axisB,
							double lambda = 0, double mu = 0);

			ParametricPoint(ParametricPoint &other, const Eigen::Matrix<double, 2, 1> &expectedPixel);

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
			 * @get The world direction of the second axis.
			 */
			const Eigen::Matrix<double, 3, 1> &getAxisB() const;

			/**
			 * @get The length of the first axis.
			 */
			double *getLambda() const;

			/**
			 * @get The length of the second axis.
			 */
			double *getMu() const;

			/**
			 * @get
			 */
			const Eigen::Matrix<double, 2, 1> &getExpectedPixel() const;

			/**
			 * @set
			 */
			void setExpectedPixel(const Eigen::Matrix<double, 2, 1> &expectedPixel);

			/**
			 * @get
			 */
			bool hasExpectedPixel() const;

			/**
			 * Factory for a [x, y, z] world point.
			 *
			 * @param expectedPixel The expected pixel.
			  * @param worldPosition The world position of the point.
			 */
			static ParametricPoint
			onPoint(const Eigen::Matrix<double, 2, 1> &expectedPixel, const Eigen::Matrix<double, 3, 1> &worldPosition);

			/**
			 * @copydoc
			 */
			static ParametricPoint onPoint(const Eigen::Matrix<double, 3, 1> &worldPosition);

			/**
			  * Factory for a [x, y, z] world point on a parametric line.
			  *
			 * @param expectedPixel The expected pixel.
			 * @param origin The origin of the parametricPoint.
			 * @param heading The heading of the line.
			 * @param lambda Optional distance from the origin in heading direction.
			 */
			static ParametricPoint
			onLine(const Eigen::Matrix<double, 2, 1> &expectedPixel, Eigen::Matrix<double, 3, 1> origin,
				   const Eigen::Matrix<double, 3, 1> &heading,
				   double lambda = 0);

			/**
			 * @copydoc
			 */
			static ParametricPoint
			onLine(Eigen::Matrix<double, 3, 1> origin, const Eigen::Matrix<double, 3, 1> &heading, double lambda = 0);

			/**
			  * Factory for a [x, y, z] world point on a parametric parametricPoint.
			 *
			 * @param expectedPixel The expected pixel.
			 * @param origin The origin of the parametricPoint.
			 * @param axisA One side of the parametricPoint.
			 * @param axisB Another side of the parametricPoint.
			 * @param lambda Optional distance from the origin in the first axis.
			 * @param mu Optional distance from the origin in the second axis.
			 */
			static ParametricPoint
			onPlane(const Eigen::Matrix<double, 2, 1> &expectedPixel, Eigen::Matrix<double, 3, 1> origin,
					const Eigen::Matrix<double, 3, 1> &axisA,
					const Eigen::Matrix<double, 3, 1> &axisB,
					double lambda = 0, double mu = 0);

			/**
			 * @copydoc
			 */
			static ParametricPoint
			onPlane(Eigen::Matrix<double, 3, 1> origin, const Eigen::Matrix<double, 3, 1> &axisA,
					const Eigen::Matrix<double, 3, 1> &axisB,
					double lambda = 0, double mu = 0);
		};
	}
}

#endif //CAMERASTABILIZATION_PARAMETRICPOINT_HPP
