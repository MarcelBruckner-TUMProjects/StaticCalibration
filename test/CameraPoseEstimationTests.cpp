#include "gtest/gtest.h"
#include <iostream>
#include <utility>
#include <StaticCalibration/CameraPoseEstimation.hpp>
#include <StaticCalibration/objects/DataSet.hpp>

#include "CameraTestBase.hpp"
#include "StaticCalibration/CameraPoseEstimationWithIntrinsics.hpp"
#include "StaticCalibration/camera/RenderingPipeline.hpp"

using namespace static_calibration::calibration;

namespace static_calibration {
    namespace tests {

        /**
         * Common setup for the camera tests.
         */
        class CameraPoseEstimationTests : public CameraTestBase {
        protected:

            int log = 0;

            static_calibration::objects::DataSet dataSet;

            std::shared_ptr<static_calibration::calibration::CameraPoseEstimationBase> estimator;

            /**
             * @destructor
             */
            ~CameraPoseEstimationTests() override = default;

            static void SetUpTestCase() {
                google::InitGoogleLogging("Camera Pose Estimation");
            }

            void TearDown() override {
                dataSet.clear();
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

            void addPointCorrespondence(const Eigen::Vector3d &pointInWorldSpace, const std::string &id) {
                Object worldObject(id, pointInWorldSpace, {0, 0, 0}, 0);
                ImageObject imageObject(id, {getPixel(worldObject.getOrigin())});
                dataSet.add(worldObject, imageObject);
            }

            void addSomePointCorrespondences() {
                addPointCorrespondence({0, 0, 5}, "a");
                addPointCorrespondence({0, 10, 5}, "b");
                addPointCorrespondence({0, 30, 5}, "c");
                addPointCorrespondence({0, 50, 5}, "d");
                addPointCorrespondence({0, 70, 5}, "e");

                addPointCorrespondence({4, 10, 0}, "f");
                addPointCorrespondence({-1, 30, -3}, "g");
                estimator->setDataSet(dataSet);
            }

            void assertEstimation(double maxDifference = 1e-8) {
                estimator->estimate(log > 0);
//				estimator->getLambdas();
                assertVectorsNearEqual(estimator->getTranslation(), translation, maxDifference);
                assertVectorsNearEqual(estimator->getRotation(), rotation, maxDifference);

                if (log > 1) {
                    std::cout << "Translation" << std::endl;
                    std::cout << estimator->getTranslation() << std::endl;
                    std::cout << "Rotation" << std::endl;
                    std::cout << estimator->getRotation() << std::endl;

                    std::cout << "World worldObjects" << std::endl;

                    for (const auto &point : estimator->getDataSet().getParametricPoints<static_calibration::calibration::Object>()) {
                        std::cout << point.getPosition() << std::endl << std::endl;
                    }
                }
            }

            void addPost(const Eigen::Vector3d &origin, std::string id) {
                int number = 5;
                double height = 1.5;
                Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
                Object postObject(id, origin, axis, height);
                ImageObject postImageObject(id);

                for (int i = 0; i < number; ++i) {
                    for (int w = -4; w <= 4; w += 2) {
                        postImageObject.addPixel(getPixel(origin + axis * (height / number) * i));
                    }
                }
                dataSet.add(postObject, postImageObject);
            }


            void addLane(const Eigen::Vector3d &origin, const Eigen::Vector3d &end, std::string id) {
                int number = 10;
                RoadMark laneObject(id, origin, end);
                ImageObject laneImageObject(id);

                for (int i = 0; i <= number; i++) {
                    Eigen::Vector3d point = origin + (laneObject.getLength() / 10. * i) * laneObject.getAxis();
                    laneImageObject.addPixel(getPixel(point));
                }

                dataSet.add(laneObject, laneImageObject);
            }
        };

        /**
         * Tests that the initial guess is 500m above the mean.
         */
        TEST_F(CameraPoseEstimationTests, testCalculateInitialGuess) {
            estimator = std::make_shared<static_calibration::calibration::CameraPoseEstimationWithIntrinsics>(
                    intrinsics);

            addPointCorrespondence({0, 0, 9}, "a");
            addPointCorrespondence({0, 0, -9}, "b");
            addPointCorrespondence({0, 9, 0}, "c");
            addPointCorrespondence({0, -9, 0}, "d");
            addPointCorrespondence({9, 0, 0}, "e");
            addPointCorrespondence({-9, 0, 0}, "f");

            estimator->setDataSet(dataSet);
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
            estimator = std::make_shared<static_calibration::calibration::CameraPoseEstimation>(intrinsics);
            addSomePointCorrespondences();
            assertEstimation();
        }


        /**
         * Tests that the rotational parameters are in the interval [-180, 180]
         */
        TEST_F(CameraPoseEstimationTests, testRotationInPlusMinus180) {
            assertVectorsNearEqual(
                    static_calibration::calibration::CameraPoseEstimationWithIntrinsics::clearRotation(
                            Eigen::Vector3d{720 + 270, -720 + 90, 12345}),
                    Eigen::Vector3d{-90, 90, 105});

            assertVectorsNearEqual(
                    static_calibration::calibration::CameraPoseEstimationWithIntrinsics::clearRotation(
                            Eigen::Vector3d{180, 0, -180}),
                    Eigen::Vector3d{180, 0, -180});

            assertVectorsNearEqual(
                    static_calibration::calibration::CameraPoseEstimationWithIntrinsics::clearRotation(
                            Eigen::Vector3d{90, 0, -90}),
                    Eigen::Vector3d{90, 0, -90});
        }


        /**
         * Tests that the optimization converges to the expected extrinsic parameters.
         */
        TEST_F(CameraPoseEstimationTests, testEstimationOnlyPosts) {
            estimator = std::make_shared<static_calibration::calibration::CameraPoseEstimation>(intrinsics);

            addPost({15, 4, 8}, "a");
            addPost({-2, 17, 0}, "b");
            addLane({-10, 0, 0}, {-10, 10, 0}, "c");

            estimator->setDataSet(dataSet);
            assertEstimation(1e-5);
        }
    }// namespace toCameraSpace
}// namespace static_calibration