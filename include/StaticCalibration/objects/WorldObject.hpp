//
// Created by brucknem on 08.02.21.
//

#ifndef CAMERASTABILIZATION_WORLDOBJECT_HPP
#define CAMERASTABILIZATION_WORLDOBJECT_HPP

#include "Eigen/Dense"
#include "ParametricPoint.hpp"
#include <vector>
#include <memory>

namespace static_calibration {
	namespace calibration {

		/**
		 * A world object containing of a set of points.
		 */
		class WorldObject {
		private:
			/**
			 * The
			 */
			std::vector<ParametricPoint> points;

			std::vector<ParametricPoint> centerLine;

			/**
			 * An optional id.
			 */
			std::string id;

			/**
			 * The height of the object in meters.
			 */
			double height = 0;

		public:

			/**
			 * @constructor
			 */
			explicit WorldObject() = default;

			/**
			 * @constructor
			 *
			 * @param point Initializes the object with the given point.
			 */
			explicit WorldObject(const ParametricPoint &point);

			/**
			 * @destructor
			 */
			virtual ~WorldObject() = default;

			/**
			 * Adds the given point to the object.
			 */
			void add(const ParametricPoint &point);

			/**
			 * The inverse number of points.
			 */
			double getWeight() const;

			/**
			 * @get
			 */
			const std::vector<ParametricPoint> &getPoints() const;

			/**
			 * @get
			 */
			std::vector<ParametricPoint> getPointsWithPixel() const;

			/**
			 * @get
			 */
			void calculateCenterLine();

			const std::vector<ParametricPoint> &getCenterLine() const;

			/**
			 * @get The mean of the points.
			 */
			Eigen::Vector3d getMean() const;

			/**
			 * @get
			 */
			const std::string &getId() const;

			/**
			 * @set
			 */
			void setId(const std::string &id);

			/**
			 * @get
			 */
			double getHeight() const;

			/**
			 * @set
			 */
			void setHeight(double height);

			/**
			 * @get
			 */
			int getNumPoints() const;
		};
	}
}

#endif //CAMERASTABILIZATION_WORLDOBJECT_HPP
