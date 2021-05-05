//
// Created by brucknem on 11.04.21.
//

#include "StaticCalibration/objects/YAMLExtension.hpp"

namespace YAML {

	bool convert<Eigen::Vector3d>::decode(const Node &node, Eigen::Vector3d &rhs) {
		if (!node.IsSequence() || node.size() != 3) {
			return false;
		}

		rhs.x() = node[0].as<double>();
		rhs.y() = node[1].as<double>();
		rhs.z() = node[2].as<double>();
		return true;
	}

	bool convert<Eigen::Vector2d>::decode(const Node &node, Eigen::Vector2d &rhs) {
		if (!node.IsSequence() || node.size() != 2) {
			return false;
		}

		rhs.x() = node[0].as<double>();
		rhs.y() = node[1].as<double>();
		return true;
	}
}