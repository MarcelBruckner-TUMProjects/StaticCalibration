//
// Created by brucknem on 04.02.21.
//

#include "StaticCalibration/objects/ImageObject.hpp"
#include "StaticCalibration/objects/DataSet.hpp"
#include "gtest/gtest.h"
#include "yaml-cpp/yaml.h"

using namespace static_calibration::calibration;

namespace static_calibration {
    namespace tests {

        /**
         * Tests for the residual blocks.
         */
        class ObjectsLoadingTests : public ::testing::Test {
        protected:

            /**
             * @destructor
             */
            ~ObjectsLoadingTests() override = default;

            void SetUp() override {
                Test::SetUp();
            }
        };

        void assertVectorEqual(const Eigen::Vector3d &vector, double x, double y, double z) {
            EXPECT_NEAR(vector.x(), x, 1e-8);
            EXPECT_NEAR(vector.y(), y, 1e-8);
            EXPECT_NEAR(vector.z(), z, 1e-8);
        }

        void assertVectorEqual(const Eigen::Vector3d &vector, const Eigen::Vector3d &other) {
            assertVectorEqual(vector, other.x(), other.y(), other.z());
        }

        void assertVectorEqual(const Eigen::Vector2d &vector, double x, double y) {
            EXPECT_NEAR(vector.x(), x, 1e-8);
            EXPECT_NEAR(vector.y(), y, 1e-8);
        }

        /**
         * Tests loading the objects from a YAML file.
         */
        TEST_F(ObjectsLoadingTests, testLoadingObjects) {
            auto dataSet = static_calibration::objects::DataSet::from<calibration::WorldObject>(
                    "../misc/objects.yaml");
            auto objects = dataSet.getWorldObjects();

            ASSERT_EQ(objects.size(), 622);
            ASSERT_EQ(dataSet.getParametricPoints().size(), 0);

            auto object = objects[0];
            ASSERT_STREQ(object.getId().c_str(), "4003002");
            ASSERT_EQ(object.getLength(), 1.21);

            assertVectorEqual(object.getOrigin(), -824.04155184631236, 851.03803717531264, -1.5656829808812063);
            assertVectorEqual(object.getAxisA(), Eigen::Vector3d::UnitZ());
        }

        /**
         * Tests loading the image objects from a YAML file.
         */
        TEST_F(ObjectsLoadingTests, testLoadingImageObjects) {
            auto dataSet = static_calibration::objects::DataSet::from<ImageObject>(
                    "../misc/pixels.yaml");
            auto imageObjects = dataSet.getImageObjects();

            ASSERT_EQ(imageObjects.size(), 62);
            ASSERT_EQ(dataSet.getParametricPoints().size(), 0);

            auto imageObject = imageObjects[0];
            ASSERT_STREQ(imageObject.getId().c_str(), "1");
            ASSERT_EQ(imageObject.size(), 3196);
            assertVectorEqual(imageObject.getPixels()[0], 487, 1200 - 1036 - 1);
            assertVectorEqual(imageObject.getPixels()[imageObject.getPixels().size() - 1], 477, 1200 - 1123 - 1);

            const std::vector<Eigen::Vector2d> &centerLine = imageObject.getCenterLine();
            ASSERT_EQ(centerLine.size(), 1123 - 1036 + 1);
            assertVectorEqual(centerLine[0], 476, 1200 - 1123 - 1);
            assertVectorEqual(centerLine[1], 482.470588235, 1200 - 1122 - 1);
            assertVectorEqual(centerLine[centerLine.size() - 1], 504.31818181818181, 1200 - 1036 - 1);
        }


        /**
         * Tests loading the image objects from a YAML file.
         */
        TEST_F(ObjectsLoadingTests, testMergeObjects) {
            auto dataset = static_calibration::objects::DataSet("../misc/objects.yaml", "../misc/pixels.yaml",
                                                                "../misc/mapping.yaml");
            const auto &parametricPoints = dataset.getParametricPoints();
            ASSERT_EQ(parametricPoints.size(), 172);
        }
    }
}

