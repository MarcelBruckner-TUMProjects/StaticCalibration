//
// Created by brucknem on 18.08.21.
//

#include "StaticCalibration/objects/DataSet.hpp"
#include "yaml-cpp/yaml.h"

#include <utility>
#include <iostream>
#include "StaticCalibration/objects/YAMLExtension.hpp"

namespace static_calibration {
    namespace objects {

        const std::vector<static_calibration::calibration::WorldObject> &DataSet::getWorldObjects() const {
            return worldObjects;
        }

        const std::vector<static_calibration::calibration::ImageObject> &DataSet::getImageObjects() const {
            return imageObjects;
        }

        template<>
        void DataSet::add(const calibration::WorldObject &object) {
            worldObjects.emplace_back(object);
        }

        template<>
        void DataSet::add(const calibration::ImageObject &object) {
            imageObjects.emplace_back(object);
        }

        YAML::Node loadFile(const std::string &objectsFile) {
            try {
                return YAML::LoadFile(objectsFile);
            } catch (const YAML::Exception &e) {
                throw std::invalid_argument("Couldn't parse " + objectsFile + "\n" + e.what());
            }
        }

        std::vector<calibration::WorldObject> loadWorldObjects(const std::string &objectsFile) {
            std::vector<calibration::WorldObject> worldObjects;
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
            }
            return worldObjects;
        }


        std::map<std::string, std::string> loadMapping(const std::string &objectsFile) {
            std::map<std::string, std::string> mapping;
            YAML::Node objectsFileYAML = loadFile(objectsFile);
            for (const auto &node : objectsFileYAML) {
                mapping[node.first.as<std::string>()] = node.second.as<std::string>();
            }
            return mapping;
        }

        std::vector<calibration::ImageObject> loadImageObjects(const std::string &objectsFile) {
            std::vector<calibration::ImageObject> imageObjects;
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
            }
            return imageObjects;
        }

        DataSet::DataSet(const std::string &objectsFile, const std::string &imageObjectsFile,
                         const std::string &mappingFile) : DataSet(
                loadWorldObjects(objectsFile),
                loadImageObjects(imageObjectsFile),
                loadMapping(mappingFile)
        ) {}

        void DataSet::clear() {
            worldObjects.clear();
            imageObjects.clear();
            mapping.clear();
        }

        void DataSet::add(const calibration::WorldObject &worldObject, const calibration::ImageObject &imageObject) {
            worldObjects.emplace_back(worldObject);
            imageObjects.emplace_back(imageObject);
            mapping[worldObject.getId()] = imageObject.getId();
            merge(worldObjects.size() - 1, imageObjects.size() - 1);
        }

        DataSet::DataSet(const std::vector<static_calibration::calibration::WorldObject> &worldObjects,
                         const std::vector<static_calibration::calibration::ImageObject> &imageObjects,
                         const std::map<std::string, std::string> &mapping) : worldObjects(worldObjects),
                                                                              imageObjects(imageObjects),
                                                                              mapping(mapping) {
            merge();
        }

        template<>
        DataSet DataSet::from<calibration::WorldObject>(const std::string &objectsFile) {
            return {
                    loadWorldObjects(objectsFile),
                    {},
                    {}
            };
        }

        const std::vector<calibration::ParametricPoint> &DataSet::getParametricPoints() const {
            return parametricPoints;
        }


        template<>
        int DataSet::get<calibration::WorldObject>(std::string id) const {
            auto objectPtr = std::find_if(worldObjects.begin(), worldObjects.end(),
                                          [&id](const calibration::WorldObject &element) {
                                              return element.getId() == id;
                                          });
            int i = int(objectPtr - worldObjects.begin());
            if (i >= worldObjects.size()) {
                i = -1;
            }
            return i;
        }

        template<>
        int DataSet::get<calibration::ImageObject>(std::string id) const {
            auto objectPtr = std::find_if(imageObjects.begin(), imageObjects.end(),
                                          [&id](const calibration::ImageObject &element) {
                                              return element.getId() == id;
                                          });
            int i = int(objectPtr - imageObjects.begin());
            if (i >= imageObjects.size()) {
                i = -1;
            }
            return i;
        }

        void DataSet::merge(int worldObjectIndex, int imageObjectIndex) {
            if (worldObjectIndex < 0 || imageObjectIndex < 0) {
                return;
            }
            for (const auto &pixel : imageObjects[imageObjectIndex].getCenterLine()) {
                parametricPoints.emplace_back(calibration::ParametricPoint(
                        pixel,
                        worldObjects[worldObjectIndex].getOrigin(),
                        worldObjects[worldObjectIndex].getAxisA(),
                        worldObjects[worldObjectIndex].getLength()
                ));
            }
        }

        void DataSet::merge() {
            parametricPoints.clear();
            for (const auto &entry : mapping) {
                auto worldObjectPtr = get<calibration::WorldObject>(entry.first);
                auto imageObjectPtr = get<calibration::ImageObject>(entry.second);

                merge(worldObjectPtr, imageObjectPtr);
            }
        }

        const std::map<std::string, std::string> &DataSet::getMapping() const {
            return mapping;
        }

        template<>
        DataSet DataSet::from<calibration::ImageObject>(const std::string &objectsFile) {
            return {
                    {},
                    loadImageObjects(objectsFile),
                    {}
            };
        }
    }
}
