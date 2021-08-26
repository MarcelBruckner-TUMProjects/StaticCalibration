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
        class DataSetTests : public ::testing::Test {
        protected:

            Eigen::Vector3d translation = Eigen::Vector3d(0, 0, 0);

            Eigen::Vector3d rotation = Eigen::Vector3d(90, 0, 0);

            std::vector<double> intrinsics = static_calibration::camera::getBlenderCameraIntrinsics();

            /**
             * @destructor
             */
            ~DataSetTests() override = default;

            void SetUp() override {
                Test::SetUp();
            }

            objects::DataSet createMockDataSetForMapping() {
                auto dataset = static_calibration::objects::DataSet();


                Eigen::Vector3d roadMark(0, 10, 0);
                auto pixel = static_calibration::camera::render(translation.data(), rotation.data(), intrinsics.data(),
                                                                roadMark.data());

                dataset.add(RoadMark("a", roadMark, roadMark));
                dataset.add(ImageObject("b", {pixel}));
                dataset.add(ImageObject("d", {pixel + Eigen::Vector2d(100, 0)}));
                dataset.add(ImageObject("c", {pixel + Eigen::Vector2d(0, 200)}));

                roadMark = Eigen::Vector3d(0, 10, 10);
                pixel = static_calibration::camera::render(translation.data(), rotation.data(), intrinsics.data(),
                                                           roadMark.data());
                dataset.add(RoadMark("0", roadMark, roadMark));
                dataset.add(ImageObject("1", {pixel}));
                dataset.add(ImageObject("2", {pixel + Eigen::Vector2d(0, 200)}));
                dataset.add(ImageObject("3", {pixel + Eigen::Vector2d(100, 0)}));

                dataset.add(ImageObject("x", {Eigen::Vector2d(0, 0)}));

                return dataset;
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
        TEST_F(DataSetTests, testLoadingObjects) {
            auto dataSet = static_calibration::objects::DataSet("../misc/objects.yaml", "", "", "");
            auto objects = dataSet.get<Object>();

            ASSERT_EQ(objects.size(), 622);
            ASSERT_EQ(dataSet.getParametricPoints<static_calibration::calibration::Object>().size(), 0);

            auto object = objects[0];
            ASSERT_STREQ(object.getId().c_str(), "4003002");
            ASSERT_EQ(object.getLength(), 1.21);

            assertVectorEqual(object.getOrigin(), -824.04155184631236, 851.03803717531264, -1.5656829808812063);
            assertVectorEqual(object.getAxis(), Eigen::Vector3d::UnitZ());
        }

        /**
         * Tests loading the image objects from a YAML file.
         */
        TEST_F(DataSetTests, testLoadingImageObjects) {
            auto dataSet = static_calibration::objects::DataSet("", "", "../misc/pixels.yaml", "");
            auto imageObjects = dataSet.get<ImageObject>();

            ASSERT_EQ(imageObjects.size(), 62);
            ASSERT_EQ(dataSet.getParametricPoints<static_calibration::calibration::Object>().size(), 0);

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
        TEST_F(DataSetTests, testLoadingExplicitRoadMarks) {
            auto dataSet = static_calibration::objects::DataSet("", "../misc/road_marks.yaml", "", "");
            auto roadMarks = dataSet.get<RoadMark>();
            ASSERT_EQ(roadMarks.size(), 3764);

            auto roadMark = roadMarks[0];
            ASSERT_EQ(roadMark.getId(), "0");
            ASSERT_EQ(roadMark.getOrigin(),
                      Eigen::Vector3d(-878.08065099595115, 858.9148813476786, -1.5917528217373729));
            ASSERT_EQ(roadMark.getEnd(), Eigen::Vector3d(-881.07365756935906, 859.10607462283224, -1.6045955746546383));
        }

        /**
         * Tests loading the image objects from a YAML file.
         */
        TEST_F(DataSetTests, testMergeObjects) {
            auto dataset = static_calibration::objects::DataSet("../misc/objects.yaml", "../misc/road_marks.yaml",
                                                                "../misc/pixels.yaml", "../misc/mapping.yaml");
            auto parametricPoints = dataset.getParametricPoints<static_calibration::calibration::Object>();
            ASSERT_EQ(parametricPoints.size(), 206);
            parametricPoints = dataset.getParametricPoints<static_calibration::calibration::RoadMark>();
            ASSERT_EQ(parametricPoints.size(), 0);
        }


        /**
         * Tests loading the image objects from a YAML file.
         */
        TEST_F(DataSetTests, testExtendMapping) {
            auto dataset = createMockDataSetForMapping();

            auto extendedMapping = dataset.extendMapping(translation, rotation, intrinsics, 210, 3);

            ASSERT_EQ(extendedMapping.size(), 2);
            auto values = std::vector<std::string>{"b", "d", "c"};
            ASSERT_EQ(extendedMapping["a"], values);
            values = std::vector<std::string>{"1", "3", "2"};
            ASSERT_EQ(extendedMapping["0"], values);

            extendedMapping = dataset.extendMapping(translation, rotation, intrinsics, 120, 3);

            ASSERT_EQ(extendedMapping.size(), 2);
            values = std::vector<std::string>{"b", "d"};
            ASSERT_EQ(extendedMapping["a"], values);
            values = std::vector<std::string>{"1", "3"};
            ASSERT_EQ(extendedMapping["0"], values);

            extendedMapping = dataset.extendMapping(translation, rotation, intrinsics, 210, 1);

            ASSERT_EQ(extendedMapping.size(), 2);
            values = std::vector<std::string>{"b"};
            ASSERT_EQ(extendedMapping["a"], values);
            values = std::vector<std::string>{"1"};
            ASSERT_EQ(extendedMapping["0"], values);
        }


        /**
         * Tests loading the image objects from a YAML file.
         */
        TEST_F(DataSetTests, testAllPossibleMappings) {
            auto dataset = createMockDataSetForMapping();

            auto mappings = dataset.createAllMappings(translation, rotation, intrinsics, 210, 3);
            ASSERT_EQ(mappings.size(), 16);
            ASSERT_TRUE(mappings[0].empty());

            ASSERT_EQ(mappings[4]["0"], "1");
            ASSERT_EQ(mappings[4]["a"], "d");

            ASSERT_EQ(mappings[13]["a"], "b");

            mappings = dataset.createAllMappings(translation, rotation, intrinsics, 210, 3, 1);
            ASSERT_EQ(mappings.size(), 7);
            ASSERT_TRUE(mappings[0].empty());

            ASSERT_EQ(mappings[4]["a"], "b");
        }
    }
}

