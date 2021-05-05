//
// Created by brucknem on 11.04.21.
//

#ifndef CAMERASTABILIZATION_YAMLEXTENSION_HPP
#define CAMERASTABILIZATION_YAMLEXTENSION_HPP

#include <Eigen/Core>
#include "yaml-cpp/yaml.h"

namespace YAML {

	/**
	 * Extension to the YAML parser for Eigen vectors.
	 */
	template<>
	struct convert<Eigen::Vector3d> {

		/**
		 * Decoder function from the YAML node to the Eigen vector.
		 */
		static bool decode(const Node &node, Eigen::Vector3d &vector);
	};

	/**
	 * Extension to the YAML parser for Eigen vectors.
	 */
	template<>
	struct convert<Eigen::Vector2d> {

		/**
		 * Decoder function from the YAML node to the Eigen vector.
		 */
		static bool decode(const Node &node, Eigen::Vector2d &vector);
	};

}
#endif //CAMERASTABILIZATION_YAMLEXTENSION_HPP
