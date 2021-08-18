//
// Created by brucknem on 17.08.21.
//

#include "StaticCalibration/utils/RenderUtils.hpp"


namespace static_calibration {
    namespace utils {

        void renderLine(cv::Mat &finalFrame, const std::string &text, int x, int y, double size) {
            cv::putText(finalFrame, text, {x, y}, cv::FONT_HERSHEY_SIMPLEX, size, {1, 1, 0});
        }

        void renderText(cv::Mat &finalFrame, std::stringstream &ss, int run) {
            int lineHeight = 34;
            cv::rectangle(finalFrame, {0, finalFrame.rows - lineHeight * 3 - 10}, {500, finalFrame.rows}, {0, 0, 0},
                          -1);
            cv::rectangle(finalFrame, {finalFrame.cols - 800 - 10, finalFrame.rows - lineHeight * 12 - 10},
                          {finalFrame.cols, finalFrame.rows}, {0, 0, 0},
                          -1);

            renderLine(finalFrame, "Run: " + std::to_string(run), 5, finalFrame.rows - 2 * lineHeight - 10);
            renderLine(finalFrame, "GREEN dots: Mapped objects", 5, finalFrame.rows - 1 * lineHeight - 10);
            renderLine(finalFrame, "RED dots: Unmapped objects", 5, finalFrame.rows - 0 * lineHeight - 10);

            std::string line;
            int i = 0;
            while (getline(ss, line)) {
                renderLine(finalFrame, line, finalFrame.cols - 800,
                           finalFrame.rows - lineHeight * 11 + i++ * lineHeight - 10);
            }
        }

        void renderText(cv::Mat &finalFrame, const static_calibration::calibration::CameraPoseEstimationBase *estimator,
                        int run) {
            std::stringstream ss;
            ss << estimator;
            renderText(finalFrame, ss, run);
        }

        void
        renderText(cv::Mat &finalFrame, const Eigen::Vector3d &translation, const Eigen::Vector3d &rotation, int run) {
            std::stringstream ss;
            ss << std::fixed;
            ss << std::setprecision(3);
            ss << "T: " << translation[0] << ", " << translation[1] << ", " << translation[2];
            ss << std::endl;
            ss << "R: " << rotation[0] << ", " << rotation[1] << ", " << rotation[2];
            renderText(finalFrame, ss, run);
        }

        void render(cv::Mat &finalFrame, const std::string &id, const Eigen::Vector3d &object,
                    const Eigen::Vector3d &translation, const Eigen::Vector3d &rotation,
                    const std::vector<double> &intrinsics, const cv::Vec3d &color, bool showId) {
            Eigen::Vector4d vectorInCameraSpace = static_calibration::camera::toCameraSpace(
                    translation.data(), rotation.data(), object.data());

            if (std::abs(vectorInCameraSpace.z()) > 1000) {
                return;
            }

            bool flipped;
            auto pixel = render(translation, rotation,
                                intrinsics, object.homogeneous(), color,
                                finalFrame, flipped);

            if (flipped) {
                return;
            }

            std::stringstream ss;
            ss << std::fixed;
            ss << id;
            //		ss << ": " << x << "," << y << "," << z;

            if (showId) {
                renderLine(finalFrame, ss.str(), (int) pixel.x(), (int) (finalFrame.rows - 1 - pixel.y()), 0.5);
            }
        }

        Eigen::Matrix<double, 2, 1> render(const Eigen::Vector3d &translation, const Eigen::Vector3d &rotation,
                                           const std::vector<double> &intrinsics,
                                           const Eigen::Vector4d &vector,
                                           const cv::Vec3d &color, cv::Mat &image) {
            bool flipped;
            return render(translation, rotation, intrinsics, vector, color, image, flipped);
        }

        void render(const cv::Vec3d &color, const cv::Mat &image, const Eigen::Vector2d &pointInImageSpace);

        Eigen::Matrix<double, 2, 1> render(const Eigen::Vector3d &translation, const Eigen::Vector3d &rotation,
                                           const std::vector<double> &intrinsics,
                                           const Eigen::Vector4d &vector,
                                           const cv::Vec3d &color, cv::Mat &image, bool &flipped) {

            Eigen::Vector2d pointInImageSpace = static_calibration::camera::render(
                    translation.data(), rotation.data(), intrinsics.data(),
                    vector.data(), flipped);

            if (flipped) {
                return pointInImageSpace;
            }
            render(color, image, pointInImageSpace);

            return pointInImageSpace;
        }

        void render(const cv::Vec3d &color, const cv::Mat &image, const Eigen::Vector2d &pointInImageSpace) {
            int imageHeight = image.rows - 1;

            cv::circle(image,
                       {(int) pointInImageSpace.x(), (int) (imageHeight - pointInImageSpace.y())},
                       std::min(5, std::max(0, (int) (image.rows * 0.01))),
                       color,
                       cv::FILLED
            );
        }

        void render(cv::Mat &finalFrame, const std::vector<static_calibration::calibration::WorldObject> &objects,
                    const Eigen::Vector3d &translation, const Eigen::Vector3d &rotation,
                    const std::vector<double> &intrinsics, bool showIds) {
            for (const auto &worldObject: objects) {
                const auto &origin = worldObject.getOrigin();
                const auto &endAxisA = worldObject.getEndAxisA();

                render(finalFrame, worldObject.getId(), origin, translation, rotation, intrinsics, {0, 0, 1}, showIds);
                render(finalFrame, worldObject.getId(), endAxisA, translation, rotation, intrinsics, {0, 0, 1}, false);
            }
        }

        void render(cv::Mat &finalFrame, const std::vector<static_calibration::calibration::ImageObject> &objects,
                    bool showIds) {
            for (const auto &imageObject: objects) {
                const auto &pixels = imageObject.getPixels();
                for (const auto &pixel : pixels) {
                    render({1, 0, 0}, finalFrame, pixel);
                }

                if (showIds && !pixels.empty()) {
                    std::stringstream ss;
                    ss << std::fixed;
                    ss << imageObject.getId();
                    auto pixel = pixels[0];
                    renderLine(finalFrame, ss.str(), (int) pixel.x(), (int) (finalFrame.rows - 1 - pixel.y()), 0.5);
                }
            }
        }

        cv::Mat addAlphaChannel(const cv::Mat &mat) {
            std::vector<cv::Mat> matChannels;
            cv::split(mat, matChannels);

            // create alpha channel
            cv::Mat alpha = cv::Mat::ones(mat.size(), CV_8UC1);
            matChannels.push_back(alpha);

            cv::Mat result;
            cv::merge(matChannels, result);
            result.convertTo(result, CV_64FC4, 1. / 255.);
            return result;
        }

        std::vector<double> translationToROStf2(std::vector<double> translation, bool inverse) {
            std::vector<double> result;

            if (!inverse) {
                result.emplace_back(translation[1]);
                result.emplace_back(-translation[0]);
                result.emplace_back(translation[2]);
            } else {
                result.emplace_back(-translation[1]);
                result.emplace_back(translation[0]);
                result.emplace_back(translation[2]);
            }

            return result;
        }

        Eigen::Vector3d translationToROStf2(const Eigen::Vector3d &translation, bool inverse) {
            std::vector<double> tmp;
            for (int i = 0; i < 3; ++i) {
                tmp.emplace_back(translation.data()[i]);
            }
            return Eigen::Vector3d(translationToROStf2(tmp, inverse).data());
        }

        std::vector<double> rotationToROStf2(std::vector<double> rotation, bool inverse) {
            std::vector<double> result;

            if (!inverse) {
                result.emplace_back(rotation[2]);
                result.emplace_back(90. - rotation[0]);
                result.emplace_back(rotation[1]);
            } else {
                result.emplace_back(90 - rotation[1]);
                result.emplace_back(rotation[2]);
                result.emplace_back(rotation[0]);
            }

            return result;
        }

        Eigen::Vector3d rotationToROStf2(const Eigen::Vector3d &rotation, bool inverse) {
            std::vector<double> tmp;
            for (int i = 0; i < 3; ++i) {
                tmp.emplace_back(rotation.data()[i]);
            }
            return Eigen::Vector3d(rotationToROStf2(tmp, inverse).data());
        }
    }
}