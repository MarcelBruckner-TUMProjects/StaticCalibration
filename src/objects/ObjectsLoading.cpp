//
// Created by brucknem on 04.03.21.
//

#include "StaticCalibration/objects/ObjectsLoading.hpp"
#include "StaticCalibration/objects/YAMLExtension.hpp"
#include "StaticCalibration/objects/ParametricPoint.hpp"

namespace static_calibration {
	namespace calibration {
		YAML::Node loadYAML(const std::string &filename) {
			size_t l = filename.length();
			if (
				filename.at(l - 5) != '.' ||
				filename.at(l - 4) != 'y' ||
				filename.at(l - 3) != 'a' ||
				filename.at(l - 2) != 'm' ||
				filename.at(l - 1) != 'l'
				) {
				throw std::invalid_argument(filename + " is not a YAML file.");
			}
			return YAML::LoadFile(filename);
		}

		bool addPixels(WorldObject &worldObject, const Eigen::Vector3d &worldPosition, int imageHeight,
					   const YAML::detail::iterator_value &imageObject) {
			bool hasPixels = false;
			for (const auto pixelNode : imageObject["pixels"]) {
				Eigen::Vector2d pixel = pixelNode.as<Eigen::Vector2d>();
				if (imageHeight > 1) {
					pixel = {pixel.x(), imageHeight - 1 - pixel.y()};
				}
				worldObject.add(
					ParametricPoint::onLine(pixel, worldPosition, Eigen::Vector3d::UnitZ()));
				hasPixels = true;
			}
			return hasPixels;
		}

		WorldObject
		createWorldObject(const YAML::detail::iterator_value &object, const YAML::Node &imageObjects, Eigen::Vector2i
		imageSize) {
			WorldObject worldObject;
			std::string objectId = object["id"].as<std::string>();
			worldObject.setId(objectId);
			worldObject.setHeight(object["height"].as<double>());
			std::string coordinateSystem = "mollweide";
			coordinateSystem = "utm";
			Eigen::Vector3d worldPosition = object[coordinateSystem + "_coord"].as<Eigen::Vector3d>();
//			worldPosition = {worldPosition.y(), -worldPosition.x(), -worldPosition.z()};

			bool hasPixels = false;
			for (const auto imageObject : imageObjects) {
				std::string frameObjectId = imageObject["id"].as<std::string>();
				if (frameObjectId == worldObject.getId()) {
					hasPixels = addPixels(worldObject, worldPosition, imageSize.y(), imageObject);
				}
			}

			if (!hasPixels) {
				worldObject.add(ParametricPoint::onPoint(worldPosition));
			}
			return worldObject;
		}

		std::vector<WorldObject>
		loadObjects(YAML::Node opendriveObjects, const YAML::Node &imageObjects, const Eigen::Vector2i &imageSize) {
			std::vector<WorldObject> objects;
			assert(opendriveObjects["objects"].IsSequence());

			for (const auto &object : opendriveObjects["objects"]) {
				if (object["type"].as<std::string>() == "pole" &&
					object["name"].as<std::string>() == "permanentDelineator") {
					objects.emplace_back(createWorldObject(object, imageObjects, imageSize));
				}
			}
			return objects;
		}

		std::vector<WorldObject>
		loadObjects(const std::string &opendriveObjectsFile, const std::string &imageObjectsFile,
					const Eigen::Vector2i &imageSize) {
			YAML::Node opendriveObjects = loadYAML(opendriveObjectsFile);
			YAML::Node imageObjects = loadYAML(imageObjectsFile);
			return loadObjects(opendriveObjects, imageObjects, imageSize);
		}
	}
}