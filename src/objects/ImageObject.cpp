//
// Created by brucknem on 17.08.21.
//

#include "StaticCalibration/objects/ImageObject.hpp"

#include <utility>

namespace static_calibration {
    namespace calibration {
        ImageObject::ImageObject(std::string id, std::vector<Eigen::Vector2d> pixels) : pixels(std::move(pixels)),
                                                                                        id(std::move(id)) {}

        ImageObject::ImageObject(std::string id) : id(std::move(id)) {}

        const std::vector<Eigen::Vector2d> &ImageObject::getPixels() const {
            return pixels;
        }

        const std::string &ImageObject::getId() const {
            return id;
        }

        void ImageObject::addPixel(const Eigen::Vector2d &pixel) {
            pixels.emplace_back(pixel);
        }

        size_t ImageObject::size() const {
            return pixels.size();
        }

        std::vector<Eigen::Vector2d> ImageObject::getCenterLine() const {
            std::vector<Eigen::Vector2d> centerLine;

            auto sortedPixels = this->pixels;
            std::sort(sortedPixels.begin(), sortedPixels.end(), [](Eigen::Vector2d lhs, Eigen::Vector2d rhs) {
                return lhs.y() < rhs.y();
            });

            std::vector<Eigen::Vector2d> currentRow = {};
            for (auto &pixel: sortedPixels) {
                if (!currentRow.empty() && pixel.y() != currentRow[0].y()) {
                    Eigen::Vector2d rowMean = {0, 0};
                    for (const auto &rowElement: currentRow) {
                        rowMean += rowElement;
                    }
                    rowMean /= (int) currentRow.size();
                    centerLine.emplace_back(rowMean);
                    currentRow.clear();
                }
                currentRow.emplace_back(pixel);
            }
            Eigen::Vector2d rowMean = {0, 0};
            for (const auto &rowElement: currentRow) {
                rowMean += rowElement;
            }
            rowMean /= (int) currentRow.size();
            centerLine.emplace_back(rowMean);
            return centerLine;
        }

        Eigen::Vector2d ImageObject::getMid() const {
            Eigen::Vector2d sum(0, 0);
            for (const auto &pixel: pixels) {
                sum += pixel;
            }
            return sum / pixels.size();
        }
    }
}