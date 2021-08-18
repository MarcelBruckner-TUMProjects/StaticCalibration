//
// Created by brucknem on 17.08.21.
//

#ifndef STATICCALIBRATION_RENDERUTILS_HPP
#define STATICCALIBRATION_RENDERUTILS_HPP

#include <opencv2/opencv.hpp>
#include <StaticCalibration/CameraPoseEstimationBase.hpp>
#include <StaticCalibration/camera/RenderingPipeline.hpp>

namespace static_calibration {
    namespace utils {

        /**
         * Renders the given line onto the frame.
         *
         * @param finalFrame The frame to render to
         * @param text The line to render.
         * @param x The pixel X location.
         * @param y The pixel Y location.
         */
        void renderLine(cv::Mat &finalFrame, const std::string &text, int x, int y, double size = 1,
                        const cv::Vec3d &color = {1, 1, 0});

        void
        renderText(cv::Mat &finalFrame, std::stringstream &ss, int run);

        /**
         * Renders the current state of the estimator and some exemplary text onto the frame.
         *
         * @param finalFrame The frame to render to.
         * @param estimator The estimator that performs static calibration.
         * @param run The index of the current run.
         */
        void
        renderText(cv::Mat &finalFrame, const static_calibration::calibration::CameraPoseEstimationBase *estimator,
                   int run);

        void
        renderText(cv::Mat &finalFrame, const Eigen::Vector3d &translation, const Eigen::Vector3d &rotation,
                   int run);

        /**
         * Renders an object onto the frame.
         *
         * @param finalFrame The frame to render to.
         * @param id The id of the object.
         * @param object The object.
         * @param translation The translation used in the pinhole camera model to render the object.
         * @param rotation The rotation used in the pinhole camera model to render the object.
         * @param intrinsics The intrinsics used in the pinhole camera model to render the object.
         * @param color The color of the projected pixels.
         * @param showId Flag if the id should be rendered.
         */
        void render(cv::Mat &finalFrame, const std::string &id, const Eigen::Vector3d &object,
                    const Eigen::Vector3d &translation, const Eigen::Vector3d &rotation,
                    const std::vector<double> &intrinsics,
                    const cv::Vec3d &color, bool showId);

        /**
         * Renders the given objects onto the frame.
         *
         * @param finalFrame The frame to render to.
         * @param objects The objects to render.
         * @param translation The translation used in the pinhole camera model to render the objects.
         * @param rotation The rotation used in the pinhole camera model to render the objects.
         * @param intrinsics The intrinsics used in the pinhole camera model to render the objects.
         * @param showIds Flag if the ids of the objects should be shown.
         */
        void render(cv::Mat &finalFrame, const std::vector<static_calibration::calibration::WorldObject> &objects,
                    const Eigen::Vector3d &translation, const Eigen::Vector3d &rotation,
                    const std::vector<double> &intrinsics, bool showIds);

        /**
         * Renders the given objects onto the frame.
         *
         * @param finalFrame The frame to render to.
         * @param objects The objects to render.
         * @param showIds Flag if the ids of the objects should be shown.
         */
        void render(cv::Mat &finalFrame, const std::vector<static_calibration::calibration::ImageObject> &objects,
                    bool showIds);

        void render(cv::Mat &finalFrame, const static_calibration::objects::DataSet &dataSet,
                    const Eigen::Vector3d &translation, const Eigen::Vector3d &rotation,
                    const std::vector<double> &intrinsics, bool showIds);

        void
        renderMapping(const cv::Mat &finalFrame, const objects::DataSet &dataSet, const Eigen::Vector3d &translation,
                      const Eigen::Vector3d &rotation, const std::vector<double> &intrinsics);

        /**
         * Adds an alpha channel to the given frame and converts to [0..1] color range.
         *
         * @param mat The input frame.
         *
         * @return The same frame with an alpha channel.
         */
        cv::Mat addAlphaChannel(const cv::Mat &mat);

        std::vector<double> translationToROStf2(std::vector<double> translation, bool inverse = false);

        Eigen::Vector3d translationToROStf2(const Eigen::Vector3d &translation, bool inverse = false);


        std::vector<double> rotationToROStf2(std::vector<double> rotation, bool inverse = false);

        Eigen::Vector3d rotationToROStf2(const Eigen::Vector3d &rotation, bool inverse = false);


        Eigen::Matrix<double, 2, 1> render(const Eigen::Vector3d &translation, const Eigen::Vector3d &rotation,
                                           const std::vector<double> &intrinsics,
                                           const Eigen::Vector4d &vector, const cv::Vec3d
                                           &color,
                                           cv::Mat &image);

        /**
         * Renders the given vector with the given color to the given image.
         *
         * @tparam T double or ceres::Jet
         * @param translation The [x, y, z] translation of the camera in world space.
         * @param rotation The [x, y, z] euler angle rotation of the camera around the world axis.
         * @param intrinsics [focalLength, sensorWidth, sensorHeight, principalX, principalY, skew]
         * @param vector The [x, y, z, w] vector in world space.
         * @param color The color that is assigned to the expectedPixel.
         * @param image The image to which the expectedPixel will rendered.
         */
        Eigen::Matrix<double, 2, 1> render(const Eigen::Vector3d &translation, const Eigen::Vector3d &rotation,
                                           const std::vector<double> &intrinsics,
                                           const Eigen::Vector4d &vector, const cv::Vec3d
                                           &color,
                                           cv::Mat &image, bool &flipped);
    }
}


#endif //STATICCALIBRATION_RENDERUTILS_HPP
