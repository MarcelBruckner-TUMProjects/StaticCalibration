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

        template<>
        const std::vector<static_calibration::calibration::Object> &DataSet::get() const {
            return worldObjects;
        }

        template<>
        const std::vector<static_calibration::calibration::RoadMark> &DataSet::get() const {
            return explicitRoadMarks;
        }

        template<>
        const std::vector<static_calibration::calibration::ImageObject> &DataSet::get() const {
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

            for (const auto &a: originalRoadMarks) {
                for (const auto &b: originalRoadMarks) {
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

            for (const auto &roadNode: objectsFileYAML["roads"]) {
                for (const auto &laneSectionNode: roadNode["laneSections"]) {
                    for (const auto &laneNode: laneSectionNode["lanes"]) {
                        for (const auto &roadMarkNode: laneNode["explicitRoadMarks"]) {
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

            for (const auto &objectNode: objectsFileYAML["objects"]) {
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
            for (const auto &node: objectsFileYAML) {
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
            for (const auto regionNode: objectsFileYAML["regions"]) {
                calibration::ImageObject imageObject(regionNode["id"].as<std::string>());
                for (const auto &pixelNode: regionNode["pixels"]) {
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
            for (const auto &pixel: imageObjects[imageObjectIndex].getCenterLine()) {
                worldObjectsParametricPoints.emplace_back(calibration::ParametricPoint(
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
            for (const auto &pixel: imageObjects[imageObjectIndex].getCenterLine()) {
                explicitRoadMarksParametricPoints.emplace_back(calibration::ParametricPoint(
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
            merge<calibration::RoadMark>(explicitRoadMarks.size() - 1, imageObjects.size() - 1);
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

        template<>
        const std::vector<calibration::ParametricPoint> &DataSet::getParametricPoints<calibration::Object>() const {
            return worldObjectsParametricPoints;
        }

        template<>
        const std::vector<calibration::ParametricPoint> &DataSet::getParametricPoints<calibration::RoadMark>() const {
            return explicitRoadMarksParametricPoints;
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
            worldObjectsParametricPoints.clear();
            explicitRoadMarksParametricPoints.clear();
            for (const auto &entry: mapping) {
                auto imageObjectPtr = get<calibration::ImageObject>(entry.second);
                merge<calibration::Object>(get<calibration::Object>(entry.first), imageObjectPtr);
                merge<calibration::RoadMark>(get<calibration::RoadMark>(entry.first), imageObjectPtr);
            }
        }

        const std::map<std::string, std::string> &DataSet::getMapping() const {
            return mapping;
        }

        std::vector<std::map<std::string, std::string>>
        DataSet::createAllMappings(const Eigen::Vector3d &translation, const Eigen::Vector3d &rotation,
                                   const std::vector<double> &intrinsics, int maxDistance, int maxElementsInDistance,
                                   int maxElementsPerMapping) {
            auto extendedMapping = extendMapping(translation, rotation, intrinsics, maxDistance, maxElementsInDistance);
            std::vector<std::pair<std::string, std::string>> mappings;

            for (const auto &entry: extendedMapping) {
                std::string worldId = entry.first;
                for (const auto &id: entry.second) {
                    mappings.emplace_back(std::make_pair(worldId, id));
                }
            }

            std::vector<std::map<std::string, std::string>> result;
            for (const auto &subset: generateAllSubsets(mappings, maxElementsPerMapping)) {
                bool badSubset = false;
                for (int i = 0; i < subset.size(); i++) {
                    if (badSubset) {
                        break;
                    }
                    for (int j = 0; j < subset.size(); j++) {
                        if (i == j) {
                            continue;
                        }
                        if (subset[i].first == subset[j].first) {
                            badSubset = true;
                            break;
                        }
                    }
                }
                if (!badSubset) {
                    std::map<std::string, std::string> r;
                    for (const auto &entry: subset) {
                        r[entry.first] = entry.second;
                    }
                    result.emplace_back(r);
                }
            }

            return result;
        }

        template<class T>
        std::vector<std::vector<T>> DataSet::generateAllSubsets(std::vector<T> &vector, int maxDepth) {
            std::vector<T> subset;
            std::vector<std::vector<T>> res;

            generateAllSubsets(vector, res, subset, 0, 0, maxDepth);

            return res;
        }

        template<class T>
        void
        DataSet::generateAllSubsets(std::vector<T> &vector, std::vector<std::vector<T>> &result, std::vector<T> &subset,
                                    int index, int depth, int maxDepth) {
            if (maxDepth > 0 && depth > maxDepth) {
                return;
            }

            result.push_back(subset);
            for (int i = index; i < vector.size(); i++) {

                // include the vector[i] in subset.
                subset.push_back(vector[i]);

                // move onto the next element.
                generateAllSubsets(vector, result, subset, i + 1, depth + 1, maxDepth);

                // exclude the vector[i] from subset and triggers
                // backtracking.
                subset.pop_back();
            }
        }

        std::map<std::string, std::vector<std::string>>
        DataSet::extendMapping(const Eigen::Vector3d &translation, const Eigen::Vector3d &rotation,
                               const std::vector<double> &intrinsics, int maxDistance, int maxElementsInDistance) {
            std::map<std::string, std::vector<std::pair<double, std::string>>> extendedMapping;

            for (const auto &roadMark: explicitRoadMarks) {
                bool flipped;
                Eigen::Vector3d mid = roadMark.getMid();
                auto roadMarkInCameraSpace = static_calibration::camera::toCameraSpace(translation.data(),
                                                                                       rotation.data(), mid.data());
                if (roadMarkInCameraSpace.z() < 0 || roadMarkInCameraSpace.z() > 1000) {
                    continue;
                }

                auto pixel = static_calibration::camera::render(translation.data(), rotation.data(),
                                                                intrinsics.data(), mid.data(),
                                                                flipped);

                for (const auto &imageObject: imageObjects) {
                    double distance = (imageObject.getMid() - pixel).norm();
                    if (distance <= maxDistance) {
                        extendedMapping[roadMark.getId()].emplace_back(std::make_pair(distance, imageObject.getId()));
                    }
                }
            }

            std::map<std::string, std::vector<std::string>> result;
            for (auto &entry: extendedMapping) {
                std::sort(entry.second.begin(), entry.second.end(),
                          [](const auto &lhs, const auto &rhs) {
                              return lhs.first < rhs.first;
                          });
                int length = std::min(maxElementsInDistance, (int) entry.second.size());
                std::vector<std::string> elements(length);
                for (int i = 0; i < length; i++) {
                    elements[i] = entry.second[i].second;
                }
                result[entry.first] = elements;
            }

            return result;
        }

    }
}
