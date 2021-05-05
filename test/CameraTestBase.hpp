//
// Created by brucknem on 01.02.21.
//

#ifndef CAMERASTABILIZATION_CAMERATESTBASE_HPP
#define CAMERASTABILIZATION_CAMERATESTBASE_HPP

#include "StaticCalibration/camera/RenderingPipeline.hpp"
#include "Eigen/Dense"
#include "gtest/gtest.h"

namespace static_calibration {
	namespace tests {

		/**
		 * Asserts that the elements of the given vectors are not further away than the maximal difference.
		 */
		void assertVectorsNearEqual(const Eigen::VectorXd &a, const Eigen::VectorXd &b, double maxDifference = 1e-4);

		/**
		 * @overload
		 */
		void assertVectorsNearEqual(const Eigen::Vector4d &a, double x, double y, double z, double w = 1,
									double maxDifference = 1e-4);

		/**
		 * @overload
		 */
		void assertVectorsNearEqual(const Eigen::Vector3d &a, double x, double y, double z = 1,
									double maxDifference = 1e-4);

		/**
		 * @overload
		 */
		void assertVectorsNearEqual(const Eigen::Vector2d &a, double x, double y,
									double maxDifference = 1e-4);

		/**
		 * Base for tests that need camera parameters.
		 */
		class CameraTestBase : public ::testing::Test {
		protected:

			/**
			 * The itnrinsics of the pinhole camera model.
			 */
			std::vector<double> intrinsics = static_calibration::camera::getBlenderCameraIntrinsics();

			/**
			 * Some [x, y, z] translation of the camera in world space.
			 */
			Eigen::Vector3d translation{0, -10, 5};

			/**
			 * Some [x, y, z] euler angle rotation of the camera around the world axis
			 */
			Eigen::Vector3d rotation{90, 0, 0};

			/**
			 * @destructor
			 */
			~CameraTestBase() override = default;
		};
	}
}

#endif //CAMERASTABILIZATION_CAMERATESTBASE_HPP
