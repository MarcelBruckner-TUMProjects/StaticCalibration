#include "gtest/gtest.h"
#include <iostream>
#include <utility>

#include "CameraTestBase.hpp"
#include "StaticCalibration/CameraPoseEstimation.hpp"
#include "StaticCalibration/camera/RenderingPipeline.hpp"

using namespace static_calibration::calibration;

namespace static_calibration {
	namespace tests {

		/**
		 * Common setup for the camera tests.
		 */
		class CameraPoseEstimationTests : public CameraTestBase {
		protected:

			std::shared_ptr<static_calibration::calibration::CameraPoseEstimation> estimator;

			/**
			 * @destructor
			 */
			~CameraPoseEstimationTests() override = default;

			static void SetUpTestCase() {
				google::InitGoogleLogging("Camera Pose Estimation");
			}

			Eigen::Vector2d getPixel(const static_calibration::calibration::ParametricPoint &object) {
				return getPixel(object.getPosition());
			}

			Eigen::Vector2d getPixel(Eigen::Vector3d vector) {
				return static_calibration::camera::render(
					translation.data(), rotation.data(),
					intrinsics.data(),
					vector.data());
			}

			void addPointCorrespondence(const Eigen::Vector3d &pointInWorldSpace) {
				WorldObject worldObject(static_calibration::calibration::ParametricPoint::onPoint(getPixel
																							   (pointInWorldSpace),
																						   pointInWorldSpace));
				estimator->addWorldObject(worldObject);
			}

			void addSomePointCorrespondences() {
				addPointCorrespondence({0, 0, 5});
				addPointCorrespondence({0, 10, 5});
				addPointCorrespondence({0, 30, 5});
				addPointCorrespondence({0, 50, 5});
				addPointCorrespondence({0, 70, 5});

				addPointCorrespondence({4, 10, 0});
				addPointCorrespondence({-1, 30, -3});
			}

			void assertEstimation(bool log = false, double maxDifference = 1e-8) {
				estimator->estimate(log);
//				estimator->getLambdas();
				assertVectorsNearEqual(estimator->getTranslation(), translation, maxDifference);
				assertVectorsNearEqual(estimator->getRotation(), rotation, maxDifference);

				if (log) {
					std::cout << "Translation" << std::endl;
					std::cout << estimator->getTranslation() << std::endl;
					std::cout << "Rotation" << std::endl;
					std::cout << estimator->getRotation() << std::endl;

					std::cout << "World worldObjects" << std::endl;
					for (const auto &worldObject : estimator->getWorldObjects()) {
						std::cout << "Next world object" << std::endl;
						for (const auto &point : worldObject.getPoints()) {
							std::cout << point.getPosition() << std::endl << std::endl;
						}
					}
				}
			}

			void addPost(const Eigen::Vector3d &origin) {
				int number = 5;
				double height = 1.5;
				WorldObject post;
				post.setHeight(height);
				for (int i = 0; i < number; ++i) {
					for (int w = -4; w <= 4; w += 2) {
						ParametricPoint point = static_calibration::calibration::ParametricPoint::onLine(origin, {0, 0, 1},
																								  (height / number) *
																								  i);
						auto pixel = getPixel(point);
						point.setExpectedPixel(pixel + Eigen::Vector2d{w, 0});
						*point.getLambda() = 0;
						post.add(point);
					}
				}
				post.calculateCenterLine();
				estimator->addWorldObject(post);
			}
		};

		/**
		 * Tests that the initial guess is 500m above the mean.
		 */
		TEST_F(CameraPoseEstimationTests, testCalculateInitialGuess) {
			estimator = std::make_shared<static_calibration::calibration::CameraPoseEstimation>();
			estimator->guessIntrinsics(intrinsics);

			addPointCorrespondence({0, 0, 9});
			addPointCorrespondence({0, 0, -9});
			addPointCorrespondence({0, 9, 0});
			addPointCorrespondence({0, -9, 0});
			addPointCorrespondence({9, 0, 0});
			addPointCorrespondence({-9, 0, 0});

			estimator->calculateInitialGuess();

			assertVectorsNearEqual(estimator->getTranslation(), Eigen::Vector3d{0, 0, 500});
			EXPECT_LE(abs(estimator->getRotation().x()), 35);
			EXPECT_LE(abs(estimator->getRotation().y()), 35);
			EXPECT_LE(abs(estimator->getRotation().z()), 35);
		}

		/**
		 * Tests that the optimization converges to the expected extrinsic parameters.
		 */
		TEST_F(CameraPoseEstimationTests, testEstimationOnlyWorldPositions) {
			estimator = std::make_shared<static_calibration::calibration::CameraPoseEstimation>();
			estimator->guessIntrinsics(intrinsics);
			estimator->fixIntrinsics(true);
			addSomePointCorrespondences();
			assertEstimation();
		}


		/**
		 * Tests that the optimization converges to the expected extrinsic parameters.
		 */
		TEST_F(CameraPoseEstimationTests, testRotationInPlusMinus180) {
			assertVectorsNearEqual(
				static_calibration::calibration::CameraPoseEstimation::clearRotation(
					Eigen::Vector3d{720 + 270, -720 + 90, 12345}),
				Eigen::Vector3d{-90, 90, 105});

			assertVectorsNearEqual(
				static_calibration::calibration::CameraPoseEstimation::clearRotation(
					Eigen::Vector3d{180, 0, -180}),
				Eigen::Vector3d{180, 0, -180});

			assertVectorsNearEqual(
				static_calibration::calibration::CameraPoseEstimation::clearRotation(
					Eigen::Vector3d{90, 0, -90}),
				Eigen::Vector3d{90, 0, -90});
		}


		/**
		 * Tests that the optimization converges to the expected extrinsic parameters.
		 */
		TEST_F(CameraPoseEstimationTests, testEstimationOnlyLines) {
			estimator = std::make_shared<static_calibration::calibration::CameraPoseEstimation>();
			estimator->guessIntrinsics(intrinsics);
			estimator->fixIntrinsics(true);

			Eigen::Vector3d origin, axisA, axisB;
			origin << 0, 0, 0;
			axisA << 1, 0, 0;
			axisB << 0, 1, 0;

//			addPointCorrespondence({1, 2, 3});

			for (int i = 0; i < 10; ++i) {
				addPost({
							(rand() % 2000) / 100. - 10,
							(rand() % 2000) / 100.,
							(rand() % 2000) / 100.
						});
			}

			estimator->guessTranslation({0, -50, 0});
			estimator->guessRotation({80, 10, -10});
			// TODO refine
			assertEstimation(false, 1e-5);

		}
	}// namespace toCameraSpace
}// namespace static_calibration