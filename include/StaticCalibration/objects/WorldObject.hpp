//
// Created by brucknem on 08.02.21.
//

#ifndef CAMERASTABILIZATION_WORLDOBJECT_HPP
#define CAMERASTABILIZATION_WORLDOBJECT_HPP

#include "Eigen/Dense"
#include "ParametricPoint.hpp"
#include "ImageObject.hpp"
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
             * The origin of the world object.
             */
            Eigen::Vector3d origin;

            /**
             * One axis of the world object.
             */
            Eigen::Vector3d axis;

            /**
             * An optional id.
             */
            std::string id;

            /**
             * The length of the object in axisA.
             */
            double length = 0;

            std::vector<ParametricPoint> parametricPoints;

        public:

            /**
             * @constructor
             */
            WorldObject(const std::string &id, const Eigen::Vector3d &origin, const Eigen::Vector3d &axisA,
                        double length);

            /**
             * @constructor
             */
            WorldObject(const std::string &id, const Eigen::Vector3d &origin, const Eigen::Vector3d &end);

            /**
             * @destructor
             */
            virtual ~WorldObject() = default;

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
            double getLength() const;

            /**
             * @set
             */
            void setLength(double length);

            const Eigen::Vector3d &getOrigin() const;

            const Eigen::Vector3d &getAxisA() const;

            const Eigen::Vector3d &getAxisB() const;

            double getLengthB() const;

            Eigen::Vector3d getEndAxisA() const;

            Eigen::Vector3d getEndAxisB() const;
        };
    }
}

#endif //CAMERASTABILIZATION_WORLDOBJECT_HPP
