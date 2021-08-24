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

        const std::vector<static_calibration::calibration::Object> &DataSet::getWorldObjects() const {
            return worldObjects;
        }

        const std::vector<static_calibration::calibration::ImageObject> &DataSet::getImageObjects() const {
            return imageObjects;
        }

        template<>
        void DataSet::add(const calibration::Object &object) {
            worldObjects.emplace_back(object);
        }

        template<>
        void DataSet::add(const calibration::RoadMark &object) {
            explicitRoadMarks.emplace_back(object);
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

        std::vector<calibration::RoadMark>
        mergeRoadMarks(const std::vector<calibration::RoadMark> &originalRoadMarks) {
            std::vector<calibration::RoadMark> roadMarks = originalRoadMarks;

            for (const auto &a : originalRoadMarks) {
                for (const auto &b : originalRoadMarks) {
                    if ((a.getEnd() - b.getOrigin()).norm() < 0.1) {
                        // TODO remove erase, as slow
                        roadMarks.erase(std::remove(roadMarks.begin(), roadMarks.end(), a), roadMarks.end());
                        roadMarks.erase(std::remove(roadMarks.begin(), roadMarks.end(), b), roadMarks.end());
                        roadMarks.emplace_back(calibration::RoadMark(a.getId(), a.getOrigin(), b.getEnd()));
                    }
                }
            }

            return roadMarks;
        }

        std::vector<calibration::RoadMark> loadExplicitRoadMarks(const std::string &objectsFile) {
            if (objectsFile.empty()) {
                return {};
            }
            std::vector<calibration::RoadMark> roadMarks;
            YAML::Node objectsFileYAML = loadFile(objectsFile);

            for (const auto &roadNode : objectsFileYAML["roads"]) {
                for (const auto &laneSectionNode : roadNode["laneSections"]) {
                    for (const auto &laneNode : laneSectionNode["lanes"]) {
                        for (const auto &roadMarkNode : laneNode["explicitRoadMarks"]) {
                            auto id = roadMarkNode["id"].as<std::string>();
                            auto start = roadMarkNode["coordinates"][0].as<Eigen::Vector3d>();
                            auto end = roadMarkNode["coordinates"][1].as<Eigen::Vector3d>();
                            auto worldObject = calibration::RoadMark(id, start, end);
                            roadMarks.emplace_back(worldObject);
                        }
                    }
                }
            }

            return mergeRoadMarks(roadMarks);
        }


        std::vector<calibration::Object> loadWorldObjects(const std::string &objectsFile) {
            if (objectsFile.empty()) {
                return {};
            }
            std::vector<calibration::Object> worldObjects;
            YAML::Node objectsFileYAML = loadFile(objectsFile);

            for (const auto &objectNode : objectsFileYAML["objects"]) {
                if (objectNode["type"].as<std::string>() != "pole" ||
                    objectNode["name"].as<std::string>() != "permanentDelineator") {
                    continue;
                }
                auto id = objectNode["id"].as<std::string>();
                auto length = objectNode["height"].as<double>();
//                if (length > 30) {
//                    continue;
//                }
                auto origin = objectNode["shifted_coord"].as<Eigen::Vector3d>();
                auto axisA = Eigen::Vector3d::UnitZ();
                calibration::Object worldObject(id, origin, axisA, length);
                worldObjects.emplace_back(worldObject);
            }
            return worldObjects;
        }


        std::map<std::string, std::string> loadMapping(const std::string &objectsFile) {
            if (objectsFile.empty()) {
                return {};
            }
            std::map<std::string, std::string> mapping;
            YAML::Node objectsFileYAML = loadFile(objectsFile);
            for (const auto &node : objectsFileYAML) {
                mapping[node.first.as<std::string>()] = node.second.as<std::string>();
            }
            return mapping;
        }

        std::vector<calibration::ImageObject> loadImageObjects(const std::string &objectsFile) {
            if (objectsFile.empty()) {
                return {};
            }
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

        DataSet::DataSet(const std::string &objectsFile, const std::string &explicitRoadMarksFile,
                         const std::string &imageObjectsFile,
                         const std::string &mappingFile) : DataSet(
                loadWorldObjects(objectsFile),
                loadExplicitRoadMarks(explicitRoadMarksFile),
                loadImageObjects(imageObjectsFile),
                loadMapping(mappingFile)
        ) {}

        void DataSet::clear() {
            worldObjects.clear();
            imageObjects.clear();
            mapping.clear();
        }

        template<>
        void DataSet::merge<calibration::Object>(int worldObjectIndex, int imageObjectIndex) {
            if (worldObjectIndex < 0 || imageObjectIndex < 0) {
                return;
            }
            for (const auto &pixel : imageObjects[imageObjectIndex].getCenterLine()) {
                parametricPoints.emplace_back(calibration::ParametricPoint(
                        pixel,
                        worldObjects[worldObjectIndex].getOrigin(),
                        worldObjects[worldObjectIndex].getAxis(),
                        worldObjects[worldObjectIndex].getLength()
                ));
            }
        }

        template<>
        void DataSet::merge<calibration::RoadMark>(int worldObjectIndex, int imageObjectIndex) {
            if (worldObjectIndex < 0 || imageObjectIndex < 0) {
                return;
            }
            for (const auto &pixel : imageObjects[imageObjectIndex].getCenterLine()) {
                parametricPoints.emplace_back(calibration::ParametricPoint(
                        pixel,
                        explicitRoadMarks[worldObjectIndex].getOrigin(),
                        explicitRoadMarks[worldObjectIndex].getAxis(),
                        explicitRoadMarks[worldObjectIndex].getLength()
                ));
            }
        }

        template<>
        void DataSet::add(const calibration::Object &worldObject, const calibration::ImageObject &imageObject) {
            worldObjects.emplace_back(worldObject);
            imageObjects.emplace_back(imageObject);
            mapping[worldObject.getId()] = imageObject.getId();
            merge<calibration::Object>(worldObjects.size() - 1, imageObjects.size() - 1);
        }

        template<>
        void DataSet::add(const calibration::RoadMark &worldObject, const calibration::ImageObject &imageObject) {
            explicitRoadMarks.emplace_back(worldObject);
            imageObjects.emplace_back(imageObject);
            mapping[worldObject.getId()] = imageObject.getId();
            merge<calibration::RoadMark>(worldObjects.size() - 1, imageObjects.size() - 1);
        }

        DataSet::DataSet(std::vector<static_calibration::calibration::Object> worldObjects,
                         std::vector<static_calibration::calibration::RoadMark> explicitRoadMarks,
                         std::vector<static_calibration::calibration::ImageObject> imageObjects,
                         std::map<std::string, std::string> mapping) : worldObjects(std::move(worldObjects)),
                                                                       explicitRoadMarks(
                                                                               std::move(explicitRoadMarks)),
                                                                       imageObjects(std::move(imageObjects)),
                                                                       mapping(std::move(mapping)) {
            merge();
        }

        const std::vector<calibration::ParametricPoint> &DataSet::getParametricPoints() const {
            return parametricPoints;
        }


        template<>
        int DataSet::get<calibration::Object>(std::string id) const {
            auto objectPtr = std::find_if(worldObjects.begin(), worldObjects.end(),
                                          [&id](const calibration::WorldObject &element) {
                                              return element.getId() == id;
                                          });
            int i = int(objectPtr - worldObjects.begin());
            if (i < worldObjects.size()) {
                return i;
            }


            return -1;
        }

        template<>
        int DataSet::get<calibration::RoadMark>(std::string id) const {
            auto objectPtr = std::find_if(explicitRoadMarks.begin(), explicitRoadMarks.end(),
                                          [&id](const calibration::WorldObject &element) {
                                              return element.getId() == id;
                                          });
            int i = int(objectPtr - explicitRoadMarks.begin());
            if (i < explicitRoadMarks.size()) {
                return i;
            }

            return -1;
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


        void DataSet::merge() {
            parametricPoints.clear();
            for (const auto &entry : mapping) {
                auto imageObjectPtr = get<calibration::ImageObject>(entry.second);
                merge<calibration::Object>(get<calibration::Object>(entry.first), imageObjectPtr);
                merge<calibration::RoadMark>(get<calibration::RoadMark>(entry.first), imageObjectPtr);
            }
        }

        const std::map<std::string, std::string> &DataSet::getMapping() const {
            return mapping;
        }

        const std::vector<static_calibration::calibration::RoadMark> &DataSet::getExplicitRoadMarks() const {
            return explicitRoadMarks;
        }
    }
}
