//
// Created by brucknem on 08.02.21.
//

#include "StaticCalibration/objects/WorldObject.hpp"

#include <utility>

namespace static_calibration {
	namespace calibration {

		void WorldObject::add(const ParametricPoint &point) {
			points.emplace_back(point);
			calculateCenterLine();
		}

		double WorldObject::getWeight() const {
			if (points.empty()) {
				return 0;
			}
			return 1. / (double) points.size();
		}

		const std::vector<ParametricPoint> &WorldObject::getPoints() const {
			return points;
		}

		WorldObject::WorldObject(const ParametricPoint &point) {
			add(point);
		}

		Eigen::Vector3d WorldObject::getMean() const {
			Eigen::Vector3d mean{0, 0, 0};
			for (const auto &point : points) {
				mean += point.getPosition();
			}
			return mean / points.size();
		}

		const std::string &WorldObject::getId() const {
			return id;
		}

		void WorldObject::setId(const std::string &value) {
			id = value;
		}

		double WorldObject::getHeight() const {
			return height;
		}

		void WorldObject::setHeight(double value) {
			height = value;
		}

		int WorldObject::getNumPoints() const {
			return (int) points.size();
		}

		std::vector<ParametricPoint> WorldObject::getPointsWithPixel() const {
			std::vector<ParametricPoint> filtered;
			for (const auto &point : points) {
				if (point.hasExpectedPixel()) {
					filtered.emplace_back(point);
				}
			}
			return filtered;
		}

		void WorldObject::calculateCenterLine() {
			centerLine.clear();
			std::vector<ParametricPoint> filtered = getPointsWithPixel();
			if (filtered.empty()) {
				return;
			}

			std::sort(filtered.begin(), filtered.end(), [](ParametricPoint a, ParametricPoint b) {
				return a.getExpectedPixel().y() < b.getExpectedPixel().y();
			});

			std::vector<Eigen::Vector2d> currentRow = {};
			for (int i = 0; i < filtered.size(); i++) {
				if (!currentRow.empty() && filtered[i].getExpectedPixel().y() != currentRow[0].y()) {
					Eigen::Vector2d sum = {0, 0};
					for (const auto &rowElement : currentRow) {
						sum += rowElement;
					}
					sum /= currentRow.size();
					centerLine.emplace_back(ParametricPoint(filtered[0], sum));
					currentRow.clear();
				}
				currentRow.emplace_back(filtered[i].getExpectedPixel());
			}
			Eigen::Vector2d sum = {0, 0};
			for (const auto &rowElement : currentRow) {
				sum += rowElement;
			}
			sum /= currentRow.size();
			centerLine.emplace_back(ParametricPoint(filtered[0], sum));
		}

		const std::vector<ParametricPoint> &WorldObject::getCenterLine() const {
			return centerLine;
		}

	}
}