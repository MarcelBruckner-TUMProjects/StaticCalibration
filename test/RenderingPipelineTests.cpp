

#include "gtest/gtest.h"
#include <iostream>

#include "CameraTestBase.hpp"

#include "StaticCalibration/camera/RenderingPipeline.hpp"

namespace static_calibration {
    namespace tests {

        /**
         * Asserts that the camera rotation matrix is built up by the given right, up and forward vectors.
         */
        void assertRotation(Eigen::Matrix4d rotation, const Eigen::Vector3d &expectedRight,
                            const Eigen::Vector3d &expectedUp, const Eigen::Vector3d &expectedForward) {
            assertVectorsNearEqual(rotation.block<3, 1>(0, 0), expectedRight);
            assertVectorsNearEqual(rotation.block<3, 1>(0, 1), expectedUp);
            assertVectorsNearEqual(rotation.block<3, 1>(0, 2), expectedForward);
        }

        /**
         * Common toCameraSpace setup for the camera tests.
         */
        class RenderingPipelineTests : public ::testing::Test {
        protected:

            std::vector<double> intrinsics = static_calibration::camera::getBlenderCameraIntrinsics();

            Eigen::Vector3d rotation{90, 0, 0};
            Eigen::Vector3d translation{0, -10, 5};

            /**
             * A default maximal difference for vector elements.
             */
            float maxDifference = 1e-3;

            /**
             * @destructor
             */
            ~RenderingPipelineTests() override = default;

            Eigen::Vector2d render(const Eigen::Vector4d &pointInWorldSpace) {
                return static_calibration::camera::render(
                        translation.data(), rotation.data(), intrinsics.data(),
                        pointInWorldSpace.data());
            }
        };


        /**
         * Tests the camera rotation matrix that is built from the euler angles rotation vector.
         */
        TEST_F(RenderingPipelineTests, testCameraRotationMatrix) {
            Eigen::Matrix4d rotation;

            rotation = static_calibration::camera::getCameraRotationMatrix(Eigen::Vector3d(0, 0, 0).data());
            assertRotation(rotation, Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), -Eigen::Vector3d::UnitZ());

            rotation = static_calibration::camera::getCameraRotationMatrix(Eigen::Vector3d(90, 0, 0).data());
            assertRotation(rotation, Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitY());

            rotation = static_calibration::camera::getCameraRotationMatrix(Eigen::Vector3d(0, 90, 0).data());
            assertRotation(rotation, -Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitY(), -Eigen::Vector3d::UnitX());

            rotation = static_calibration::camera::getCameraRotationMatrix(Eigen::Vector3d(0, 0, 90).data());
            assertRotation(rotation, Eigen::Vector3d::UnitY(), -Eigen::Vector3d::UnitX(), -Eigen::Vector3d::UnitZ());

            rotation = static_calibration::camera::getCameraRotationMatrix(Eigen::Vector3d(-90, 0, 0).data());
            assertRotation(rotation, Eigen::Vector3d::UnitX(), -Eigen::Vector3d::UnitZ(), -Eigen::Vector3d::UnitY());

            rotation = static_calibration::camera::getCameraRotationMatrix(Eigen::Vector3d(90, 90, 0).data());
            assertRotation(rotation, -Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY());

            rotation = static_calibration::camera::getCameraRotationMatrix(Eigen::Vector3d(90, 0, 90).data());
            assertRotation(rotation, Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ(), -Eigen::Vector3d::UnitX());

            rotation = static_calibration::camera::getCameraRotationMatrix(Eigen::Vector3d(0, 90, 90).data());
            assertRotation(rotation, -Eigen::Vector3d::UnitZ(), -Eigen::Vector3d::UnitX(), -Eigen::Vector3d::UnitY());

            rotation = static_calibration::camera::getCameraRotationMatrix(Eigen::Vector3d(90, 90, 90).data());
            assertRotation(rotation, -Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitY(), -Eigen::Vector3d::UnitX());

            rotation = static_calibration::camera::getCameraRotationMatrix(Eigen::Vector3d(180, 0, 0).data());
            assertRotation(rotation, Eigen::Vector3d::UnitX(), -Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ());

            rotation = static_calibration::camera::getCameraRotationMatrix(Eigen::Vector3d(0, 180, 0).data());
            assertRotation(rotation, -Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ());

            rotation = static_calibration::camera::getCameraRotationMatrix(Eigen::Vector3d(0, 0, 180).data());
            assertRotation(rotation, -Eigen::Vector3d::UnitX(), -Eigen::Vector3d::UnitY(), -Eigen::Vector3d::UnitZ());

            rotation = static_calibration::camera::getCameraRotationMatrix(Eigen::Vector3d(180, 180, 180).data());
            assertRotation(rotation, Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), -Eigen::Vector3d::UnitZ());

            rotation = static_calibration::camera::getCameraRotationMatrix(Eigen::Vector3d(360, 360, 360).data());
            assertRotation(rotation, Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), -Eigen::Vector3d::UnitZ());
        }


        /**
         * Tests the world to camera coordinates transformation.
         */
        TEST_F(RenderingPipelineTests, testWorldToCameraTransformation) {
            Eigen::Vector4d pointInWorldSpace, pointInCameraSpace;
            pointInWorldSpace << 0, 10, 0, 1;

            pointInCameraSpace = static_calibration::camera::toCameraSpace(translation.data(), rotation.data(),
                                                                           pointInWorldSpace.data());
            assertVectorsNearEqual(pointInCameraSpace, 0, -5, 20);

            pointInWorldSpace << 10, 0, 0, 1;
            pointInCameraSpace = static_calibration::camera::toCameraSpace(translation.data(), rotation.data(),
                                                                           pointInWorldSpace.data());
            assertVectorsNearEqual(pointInCameraSpace, 10, -5, 10);

            pointInWorldSpace << 0, 0, 10, 1;
            pointInCameraSpace = static_calibration::camera::toCameraSpace(translation.data(), rotation.data(),
                                                                           pointInWorldSpace.data());
            assertVectorsNearEqual(pointInCameraSpace, 0, 5, 10);

            pointInWorldSpace << -10, -10, -10, 1;
            pointInCameraSpace = static_calibration::camera::toCameraSpace(translation.data(), rotation.data(),
                                                                           pointInWorldSpace.data());
            assertVectorsNearEqual(pointInCameraSpace, -10, -15, 0);
        }

        /**
         * Tests that rendering points in world coordinates results in correct pixels in an image.
         */
        TEST_F(RenderingPipelineTests, testRenderToImage) {
            Eigen::Vector4d pointInWorldSpace;
            Eigen::Vector2d pointInImageSpace;

            pointInWorldSpace << 0, 0, 0, 1;
            assertVectorsNearEqual(render(pointInWorldSpace), 960, 0);

            pointInWorldSpace << 7.99, 0, 0, 1;
            assertVectorsNearEqual(render(pointInWorldSpace), 1918.8, 0);
            pointInWorldSpace << 8, 0, 0, 1;
            assertVectorsNearEqual(render(pointInWorldSpace), 1920, 0);

            pointInWorldSpace << -8, 0, 0, 1;
            assertVectorsNearEqual(render(pointInWorldSpace), 0, 0, maxDifference);

            pointInWorldSpace << 0, 0, 10, 1;
            assertVectorsNearEqual(render(pointInWorldSpace), 960, 1200, maxDifference);

            pointInWorldSpace << 8, 0, 10, 1;
            assertVectorsNearEqual(render(pointInWorldSpace), 1920, 1200, maxDifference);

            pointInWorldSpace << -8, 0, 10, 1;
            assertVectorsNearEqual(render(pointInWorldSpace), 0, 1200, maxDifference);

            for (int i = -8; i < 1000; i += 10) {
                pointInWorldSpace << 0, i, 5, 1;
                assertVectorsNearEqual(render(pointInWorldSpace), 1920.f / 2, 1200.f / 2, maxDifference);
            }
        }


        /**
         * Tests that rendering points at special locations like the direct camera position, or outside the expected
         * image parametricPoint results in valid pixels.
         */
        TEST_F(RenderingPipelineTests, testRenderPointsOutOfFrustum) {
            Eigen::Vector4d pointInWorldSpace;
            Eigen::Vector2d pixel;

            pointInWorldSpace << 0, 20, 0, 1;
            pixel = render(pointInWorldSpace);
            assertVectorsNearEqual(pixel, 960, 400);

            pointInWorldSpace << 0, -20, 0, 1;
            pixel = render(pointInWorldSpace);
            assertVectorsNearEqual(pixel, 960, 1200);

            pointInWorldSpace << translation.x(), translation.y(), translation.z(), 1;
            pixel = render(pointInWorldSpace);
            assertVectorsNearEqual(pixel, 0, 0);

            pointInWorldSpace << -100, -9, 5, 1;
            pixel = render(pointInWorldSpace);
            assertVectorsNearEqual(pixel, -119040, 600);

        }

        /**
         * Tests the mock blender camera matrix.
         */
        TEST_F(RenderingPipelineTests, testGetBlenderIntrinsics) {
            auto intrinsics = static_calibration::camera::getBlenderCameraIntrinsics();

            ASSERT_EQ(intrinsics[0], 1200);
            ASSERT_EQ(intrinsics[1], 1200);

            ASSERT_EQ(intrinsics[2], 960);
            ASSERT_EQ(intrinsics[3], 600);

            ASSERT_EQ(intrinsics[4], 0);
        }
    }// namespace toCameraSpace
}// namespace static_calibration