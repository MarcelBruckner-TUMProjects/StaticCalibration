//
// Created by brucknem on 17.08.21.
//

#include "StaticCalibration/utils/RenderUtils.hpp"


namespace static_calibration {
    namespace utils {

        void
        renderLine(cv::Mat &finalFrame, const std::string &text, int x, int y, double size,
                   const cv::Vec3d &color) {
            cv::putText(finalFrame, text, {x, y}, cv::FONT_HERSHEY_SIMPLEX, size, color);
        }

        void renderText(cv::Mat &finalFrame, std::stringstream &ss, int run, int maxRuns, double evaluationError) {
            bool printRun = !(run < 0 || maxRuns < 0 || evaluationError < 0);

            int lineHeight = 34;
            double grey = .4;
            if (printRun) {
                cv::rectangle(finalFrame,
                              {0, finalFrame.rows - lineHeight * 2 - 10}, {500, finalFrame.rows}, {grey, grey, grey},
                              -1);
                renderLine(finalFrame, "Testing mapping: " + std::to_string(run) + " / " + std::to_string(maxRuns), 5,
                           finalFrame.rows - 1 * lineHeight - 10);
                renderLine(finalFrame, "Error: " + std::to_string(evaluationError), 5,
                           finalFrame.rows - 0 * lineHeight - 10);
            }

            cv::rectangle(finalFrame, {finalFrame.cols - 800 - 10, finalFrame.rows - lineHeight * 12 - 10},
                          {finalFrame.cols, finalFrame.rows}, {grey, grey, grey},
                          -1);

            std::string line;
            int i = 0;
            while (getline(ss, line)) {
                renderLine(finalFrame, line, finalFrame.cols - 800,
                           finalFrame.rows - lineHeight * 11 + i++ * lineHeight - 10);
            }
        }

        void renderText(cv::Mat &finalFrame, const static_calibration::calibration::CameraPoseEstimationBase *estimator,
                        int run, int maxRuns, double evaluationError) {
            std::stringstream ss;
            ss << estimator;
            renderText(finalFrame, ss, run, maxRuns, evaluationError);
        }

        void
        renderText(cv::Mat &finalFrame, const Eigen::Vector3d &translation, const Eigen::Vector3d &rotation) {
            std::stringstream ss;
            ss << std::fixed;
            ss << std::setprecision(3);
            ss << "T: " << translation[0] << ", " << translation[1] << ", " << translation[2];
            ss << std::endl;
            ss << "R: " << rotation[0] << ", " << rotation[1] << ", " << rotation[2];
            renderText(finalFrame, ss);
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
                renderLine(finalFrame, ss.str(), (int) pixel.x(), (int) (finalFrame.rows - 1 - pixel.y()), 0.5,
                           {0, 1, 1});
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

        void render(cv::Mat &finalFrame, const static_calibration::calibration::WorldObject &worldObject,
                    const Eigen::Vector3d &translation, const Eigen::Vector3d &rotation,
                    const std::vector<double> &intrinsics, bool showIds, int maxRenderDistance, cv::Vec3d color) {
            const auto &origin = worldObject.getOrigin();
            const auto &endAxisA = worldObject.getEnd();

            Eigen::Vector4d vectorInCameraSpace = static_calibration::camera::toCameraSpace(
                    translation.data(), rotation.data(), origin.data());

            if (vectorInCameraSpace.z() < 0 || vectorInCameraSpace.z() > maxRenderDistance) {
                return;
            }

            bool flipped;
            auto originPixel = render(translation, rotation,
                                      intrinsics, origin.homogeneous(), color,
                                      finalFrame, flipped);
            if (flipped) {
                return;
            }

            auto endPixel = render(translation, rotation,
                                   intrinsics, endAxisA.homogeneous(), color,
                                   finalFrame, flipped);
            if (flipped) {
                return;
            }

            std::stringstream ss;
            ss << std::fixed;
            ss << worldObject.getId();
            //		ss << ": " << x << "," << y << "," << z;

            if (showIds) {
                renderLine(finalFrame, ss.str(), (int) originPixel.x(),
                           (int) (finalFrame.rows - 1 - originPixel.y()), 0.5, {0, 1, 1});
            }

            cv::line(finalFrame, cv::Point(originPixel.x(), finalFrame.rows - 1 - originPixel.y()),
                     cv::Point(endPixel.x(), finalFrame.rows - 1 - endPixel.y()),
                     color, 2);
        }

        void render(cv::Mat &finalFrame, const std::vector<static_calibration::calibration::Object> &objects,
                    const Eigen::Vector3d &translation, const Eigen::Vector3d &rotation,
                    const std::vector<double> &intrinsics, bool showIds, int maxRenderDistance) {
            cv::Vec3d color(0, 0, 1);
            for (const auto &worldObject: objects) {
                render(finalFrame, worldObject, translation, rotation, intrinsics, showIds, maxRenderDistance, color);
            }
        }

        void render(cv::Mat &finalFrame, const std::vector<static_calibration::calibration::RoadMark> &objects,
                    const Eigen::Vector3d &translation, const Eigen::Vector3d &rotation,
                    const std::vector<double> &intrinsics, bool showIds, int maxRenderDistance) {
            cv::Vec3d color(1, 1, 1);
            for (const auto &worldObject: objects) {
                render(finalFrame, worldObject, translation, rotation, intrinsics, showIds, maxRenderDistance, color);
            }
        }


        void render(cv::Mat &finalFrame, const static_calibration::objects::DataSet &dataSet,
                    const Eigen::Vector3d &translation, const Eigen::Vector3d &rotation,
                    const std::vector<double> &intrinsics, bool showIds, int maxRenderDistance) {
            static_calibration::utils::render(finalFrame, dataSet.get<calibration::ImageObject>(), showIds);
            static_calibration::utils::render(finalFrame, dataSet.get<calibration::Object>(), translation, rotation,
                                              intrinsics,
                                              showIds, maxRenderDistance);
            static_calibration::utils::render(finalFrame, dataSet.get<calibration::RoadMark>(), translation, rotation,
                                              intrinsics, showIds, maxRenderDistance);
            static_calibration::utils::renderMapping(finalFrame, dataSet, translation, rotation, intrinsics);
        }

        void
        renderMapping(const cv::Mat &finalFrame, const objects::DataSet &dataSet, const Eigen::Vector3d &translation,
                      const Eigen::Vector3d &rotation, const std::vector<double> &intrinsics) {
            for (const auto &mapping: dataSet.getMergedMappings()) {
                int worldObjectIndex = dataSet.get<calibration::Object>(mapping.first);
                int roadMarkIndex = dataSet.get<calibration::RoadMark>(mapping.first);

                calibration::WorldObject worldObject;
                if (worldObjectIndex != -1) {
                    worldObject = dataSet.get<calibration::Object>()[worldObjectIndex];
                } else if (roadMarkIndex != -1) {
                    worldObject = dataSet.get<calibration::RoadMark>()[roadMarkIndex];
                } else {
                    continue;
                }
                auto imageObject = dataSet.get<calibration::ImageObject>()[dataSet.get<calibration::ImageObject>(
                        mapping.second)];

                bool flipped;
                Eigen::Vector2d pixel = camera::render(translation.data(), rotation.data(),
                                                       intrinsics.data(),
                                                       worldObject.getMid().data(),
                                                       flipped);
                if (flipped) {
                    continue;
                }
                auto centerLineSizeHalf = imageObject.getCenterLine().size() / 2;
                cv::line(finalFrame, cv::Point(pixel.x(), finalFrame.rows - 1 - pixel.y()),
                         cv::Point(imageObject.getCenterLine()[centerLineSizeHalf].x(),
                                   finalFrame.rows - 1 - imageObject.getCenterLine()[centerLineSizeHalf].y()),
                         {0, 1, 0}, 2);
            }
        }

        void render(cv::Mat &finalFrame, const std::vector<static_calibration::calibration::ImageObject> &objects,
                    bool showIds) {
            int imageHeight = finalFrame.rows - 1;

            for (const auto &imageObject: objects) {
                const auto &pixels = imageObject.getPixels();
                for (const auto &pixel: pixels) {
                    finalFrame.at<cv::Vec4d>(cv::Point((int) pixel.x(), int(imageHeight - pixel.y()))) =
                            cv::Vec4d(1, 0, 0, 1);
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

        cv::Mat removeAlphaChannel(const cv::Mat &mat) {
            std::vector<cv::Mat> matChannels;
            cv::split(mat, matChannels);

            cv::Mat result;
            cv::merge(std::vector<cv::Mat>(matChannels.begin(), matChannels.end() - 1), result);
            result.convertTo(result, CV_8UC3, 255);
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