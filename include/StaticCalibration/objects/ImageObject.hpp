//
// Created by brucknem on 17.08.21.
//

#ifndef STATICCALIBRATION_IMAGEOBJECT_HPP
#define STATICCALIBRATION_IMAGEOBJECT_HPP

#include <Eigen/Core>
#include <vector>

namespace static_calibration {
    namespace calibration {
        class ImageObject {
            std::vector<Eigen::Vector2d> pixels;

            std::string id;

        public:
            explicit ImageObject(std::string id);

            ImageObject(std::string id, std::vector<Eigen::Vector2d> pixels);

            void addPixel(const Eigen::Vector2d &pixel);

            const std::vector<Eigen::Vector2d> &getPixels() const;

            const std::string &getId() const;

            size_t size() const;

            std::vector<Eigen::Vector2d> getCenterLine() const;
        };
    }
}


#endif //STATICCALIBRATION_IMAGEOBJECT_HPP
