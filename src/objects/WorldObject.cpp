//
// Created by brucknem on 08.02.21.
//

#include "StaticCalibration/objects/WorldObject.hpp"

#include <utility>

namespace static_calibration {
    namespace calibration {

        const std::string &WorldObject::getId() const {
            return id;
        }

        void WorldObject::setId(const std::string &value) {
            id = value;
        }

        double WorldObject::getLength() const {
            return length;
        }

        void WorldObject::setLength(double value) {
            length = value;
        }

        WorldObject::WorldObject(const std::string &id, const Eigen::Vector3d &origin, const Eigen::Vector3d &axisA,
                                 double length)
                : origin(origin),
                  axis(axisA.stableNormalized()),
                  id(id),
                  length(length) {}

        const Eigen::Vector3d &WorldObject::getOrigin() const {
            return origin;
        }

        const Eigen::Vector3d &WorldObject::getAxisA() const {
            return axis;
        }

        Eigen::Vector3d WorldObject::getEndAxisA() const {
            return getOrigin() + getAxisA() * getLength();
        }

        WorldObject::WorldObject(const std::string &id, const Eigen::Vector3d &origin, const Eigen::Vector3d &end) :
                WorldObject(id, origin, (end - origin), (end - origin).stableNorm()) {}
    }
}