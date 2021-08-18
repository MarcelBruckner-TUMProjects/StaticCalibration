//
// Created by brucknem on 18.08.21.
//

#include "StaticCalibration/objects/DataSet.hpp"
#include "yaml-cpp/yaml.h"

#include <utility>
#include "StaticCalibration/objects/YAMLExtension.hpp"

namespace static_calibration {
    namespace objects {
        template<>
        void DataSet::merge(const calibration::WorldObject &object) {
            for (const auto &imageObject : imageObjects) {
                if (imageObject.getId() == object.getId()) {
                    for (const auto &pixel : imageObject.getCenterLine()) {
                        parametricPoints.emplace_back(calibration::ParametricPoint(
                                pixel,
                                object.getOrigin(),
                                object.getAxisA(),
                                object.getLength()
                        ));
                    }
                }
            }
        }

        template<>
        void DataSet::merge(const calibration::ImageObject &object) {
            for (const auto &worldObject : worldObjects) {
                if (worldObject.getId() == object.getId()) {
                    for (const auto &pixel : object.getCenterLine()) {
                        parametricPoints.emplace_back(calibration::ParametricPoint(
                                pixel,
                                worldObject.getOrigin(),
                                worldObject.getAxisA(),
                                worldObject.getLength()
                        ));
                    }
                }
            }
        }


        DataSet::DataSet(
                std::vector<static_calibration::calibration::WorldObject> worldObjects,
                std::vector<static_calibration::calibration::ImageObject> imageObjects) :
                worldObjects(std::move(worldObjects)),
                imageObjects(std::move(imageObjects)) {
            for (const auto &worldObject : this->worldObjects) {
                merge(worldObject);
            }
        }

        const std::vector<static_calibration::calibration::WorldObject> &DataSet::getWorldObjects() const {
            return worldObjects;
        }

        const std::vector<static_calibration::calibration::ImageObject> &DataSet::getImageObjects() const {
            return imageObjects;
        }


        template<>
        void DataSet::add(const calibration::WorldObject &object) {
            worldObjects.emplace_back(object);
            merge(object);
        }

        template<>
        void DataSet::add(const calibration::ImageObject &object) {
            imageObjects.emplace_back(object);
            merge(object);
        }

        YAML::Node loadFile(const std::string &objectsFile) {
            try {
                return YAML::LoadFile(objectsFile);
            } catch (const YAML::Exception &e) {
                throw std::invalid_argument("Couldn't parse " + objectsFile + "\n" + e.what());
            }
        }

        template<>
        void DataSet::loadFromFile<calibration::WorldObject>(const std::string &objectsFile) {
            YAML::Node objectsFileYAML = loadFile(objectsFile);

            for (const auto &objectNode : objectsFileYAML["objects"]) {
                if (objectNode["type"].as<std::string>() != "pole" ||
                    objectNode["name"].as<std::string>() != "permanentDelineator") {
                    continue;
                }
                auto id = objectNode["id"].as<std::string>();
                auto length = objectNode["height"].as<double>();
                auto origin = objectNode["shifted_coord"].as<Eigen::Vector3d>();
                auto axisA = Eigen::Vector3d::UnitZ();
                calibration::WorldObject worldObject(id, origin, axisA, length);
                worldObjects.emplace_back(worldObject);
                merge(worldObject);
            }
        }


        template<>
        void DataSet::loadFromFile<calibration::ImageObject>(const std::string &objectsFile) {
            YAML::Node objectsFileYAML = loadFile(objectsFile);

            auto imageHeight = objectsFileYAML["image_size"].as<std::vector<int>>()[0];
            for (const auto regionNode : objectsFileYAML["regions"]) {
                calibration::ImageObject imageObject(regionNode["id"].as<std::string>());
                for (const auto &pixelNode : regionNode["pixels"]) {
                    Eigen::Vector2d pixel = pixelNode.as<Eigen::Vector2d>();
                    if (imageHeight > 1) {
                        pixel = {pixel.x(), imageHeight - 1 - pixel.y()};
                    }
                    imageObject.addPixel(pixel);
                }
                imageObjects.emplace_back(imageObject);
                merge(imageObject);
            }
        }


        DataSet::DataSet(const std::string &objectsFile, const std::string &imageObjectsFile) {
            loadFromFile<calibration::WorldObject>(objectsFile);
            loadFromFile<calibration::ImageObject>(imageObjectsFile);
        }

        const std::vector<static_calibration::calibration::ParametricPoint> &DataSet::getParametricPoints() const {
            return parametricPoints;
        }

        void DataSet::clear() {
            worldObjects.clear();
            imageObjects.clear();
        }

        void DataSet::add(const calibration::WorldObject &worldObject, const calibration::ImageObject &imageObject) {
            worldObjects.emplace_back(worldObject);
            imageObjects.emplace_back(imageObject);
            merge(worldObject, imageObject);
        }

        void DataSet::merge(const calibration::WorldObject &worldObject, const calibration::ImageObject &imageObject) {
            for (const auto &pixel : imageObject.getCenterLine()) {
                parametricPoints.emplace_back(calibration::ParametricPoint(
                        pixel,
                        worldObject.getOrigin(),
                        worldObject.getAxisA(),
                        worldObject.getLength()
                ));
            }
        }

    }
}
