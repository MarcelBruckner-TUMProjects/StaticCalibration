//
// Created by brucknem on 02.02.21.
//
#include <iostream>
#include "CMakeConfig.h"

#ifdef WITH_OPENCV

#include <thread>
#include <boost/algorithm/string/split.hpp>
#include "Eigen/Dense"
#include "StaticCalibration/camera/RenderingPipeline.hpp"
#include "StaticCalibration/objects/WorldObject.hpp"
#include "StaticCalibration/objects/ObjectsLoading.hpp"
#include "StaticCalibration/CameraPoseEstimation.hpp"
#include "StaticCalibration/utils/CommandLineParser.hpp"
#include "CSVWriter.hpp"

#include <boost/foreach.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <utility>
#include "glog/logging.h"

//
//void initCSVWriters() {
//    auto pixelsFilePath = boost::filesystem::path(pixelsFile);
//    evaluationPath =
//            outputFolder / "StaticCalibration" / pixelsFilePath.parent_path().filename();
//    if (!boost::filesystem::is_directory(evaluationPath)) {
//        boost::filesystem::create_directories(evaluationPath);
//    }
//    auto suffix = getNowSuffix();
//    extrinsicParametersWriter = new CSVWriter(
//            evaluationPath / (pixelsFilePath.filename().string() + suffix + ".csv"));
//
//    *extrinsicParametersWriter << "Run"
//                               << "Correspondences"
//                               << "Penalize Scale [Lambdas]"
//                               << "Penalize Scale [Rotation]"
//                               << "Penalize Scale [Weights]"
//                               << "Valid Solution"
//                               << "Loss"
//                               << "Loss [Correspondences]"
//                               << "Loss [Lambdas]"
//                               << "Loss [Intrinsics]"
//                               << "Loss [Rotations]"
//                               << "Loss [Weights]"
//                               << "Translation [x]"
//                               << "Translation [y]"
//                               << "Translation [z]"
//                               << "Rotation [x]"
//                               << "Rotation [y]"
//                               << "Rotation [z]"
//                               << "Focal Length"
//                               << "Focal Length [Ratio]"
//                               << "Principal Point [u]"
//                               << "Principal Point [v]"
//                               << "Skew"
//                               << "Weights [Avg]"
//                               << "Weights [Min]"
//                               << "Weights [Max]"
//                               << newline
//                               << flush;
//}

/**
 * Renders the current state of the estimator and some exemplary text onto the frame.
 *
 * @param finalFrame The frame to render to.
 * @param estimator The estimator that performs static calibration.
 * @param run The index of the current run.
 */
void renderText(cv::Mat &finalFrame, const static_calibration::calibration::CameraPoseEstimation &estimator, int run);

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
            const Eigen::Vector3d &translation, const Eigen::Vector3d &rotation, const std::vector<double> &intrinsics,
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
 * Adds an alpha channel to the given frame and converts to [0..1] color range.
 *
 * @param mat The input frame.
 *
 * @return The same frame with an alpha channel.
 */
cv::Mat addAlphaChannel(const cv::Mat &mat);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/**
 * main
 */
int main(int argc, char const *argv[]) {
    srandom(time(nullptr));
    auto parsedOptions = static_calibration::app::parseCommandLine(argc, argv);

    auto objects = static_calibration::calibration::loadObjects(parsedOptions.objectsFile, parsedOptions.pixelsFile,
                                                                parsedOptions.imageSize);
    google::InitGoogleLogging("Static Calibration");
    static_calibration::calibration::CameraPoseEstimation estimator;
    estimator.guessIntrinsics(parsedOptions.focalLength, parsedOptions.focalLengthRatio,
                              parsedOptions.principalPoint,
                              parsedOptions.skew);

    cv::Mat evaluationFrame = cv::imread("../misc/test_frame.png");
    evaluationFrame = addAlphaChannel(evaluationFrame);
    cv::Mat finalFrame;

    bool optimizationFinished;
    Eigen::Vector3d translation;
    Eigen::Vector3d rotation;
    std::vector<double> intrinsics;

//    initCSVWriters();

    const char *windowName = "Evaluate Static Calibration";
    cv::namedWindow(windowName);
    int trackbarShowIds = 0;
    cv::createTrackbar("Show IDs", windowName, &trackbarShowIds, 1);

    int run = 0;
    while (true) {
        optimizationFinished = estimator.isEstimationFinished();
        translation = estimator.getTranslation();
        rotation = estimator.getRotation();
        intrinsics = estimator.getIntrinsics();

        if (optimizationFinished) {
            run++;
            estimator.clearWorldObjects();
            estimator.addWorldObjects(objects);
            estimator.guessIntrinsics(parsedOptions.focalLength, parsedOptions.focalLengthRatio,
                                      parsedOptions.principalPoint,
                                      parsedOptions.skew);
            optimizationFinished = false;
            estimator.estimateAsync(parsedOptions.logEstimationProgress);
        }

        finalFrame = evaluationFrame * 0.5;
        render(finalFrame, objects, translation, rotation, intrinsics, trackbarShowIds);
        renderText(finalFrame, estimator, run);

        cv::imshow(windowName, finalFrame);
        if ((char) cv::waitKey(1) == 'q') {
            break;
        }
    }

    return EXIT_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Renders the given line onto the frame.
 *
 * @param finalFrame The frame to render to
 * @param text The line to render.
 * @param x The pixel X location.
 * @param y The pixel Y location.
 */
void renderLine(cv::Mat &finalFrame, const std::string &text, int x, int y) {
    cv::putText(finalFrame, text, {x, y}, cv::FONT_HERSHEY_SIMPLEX, 1,
                {1, 1, 0});
}

void
render(cv::Mat &finalFrame, const std::string &id, const Eigen::Vector3d &object, const Eigen::Vector3d &translation,
       const Eigen::Vector3d &rotation, const std::vector<double> &intrinsics, const cv::Vec3d &color, bool showId) {
    Eigen::Vector4d vectorInCameraSpace = static_calibration::camera::toCameraSpace(
            translation.data(), rotation.data(), object.data());

    if (std::abs(vectorInCameraSpace.z()) > 2000) {
        return;
    }

    bool flipped;
    auto pixel = static_calibration::camera::render(translation, rotation,
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
        renderLine(finalFrame, ss.str(), (int) pixel.x(), (int) (finalFrame.rows - 1 - pixel.y()));
    }
}

void render(cv::Mat &finalFrame, const std::vector<static_calibration::calibration::WorldObject> &objects,
            const Eigen::Vector3d &translation, const Eigen::Vector3d &rotation,
            const std::vector<double> &intrinsics, bool showIds) {
    for (const auto &worldObject: objects) {
        bool idShown = false;
        auto centerLine = worldObject.getCenterLine();
        if (centerLine.empty()) {
            for (const auto &point : worldObject.getPoints()) {
                Eigen::Vector3d p = point.getPosition();
                cv::Vec3d color = {0, 0, 1};
                render(finalFrame, worldObject.getId(), p, translation, rotation, intrinsics, color,
                       !idShown && showIds);
                idShown = true;
            }
        }
        for (const auto &point : centerLine) {
            Eigen::Vector3d p = point.getPosition();
            cv::Vec3d color = {0, 0, 1};
            if (point.hasExpectedPixel()) {
                color = {0, 1, 0};
            }
            render(finalFrame, worldObject.getId(), p, translation, rotation, intrinsics, color, !idShown && showIds);
            idShown = true;
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


void renderText(cv::Mat &finalFrame, const static_calibration::calibration::CameraPoseEstimation &estimator, int run) {
    int lineHeight = 34;
    cv::rectangle(finalFrame, {0, finalFrame.rows - lineHeight * 3 - 10}, {500, finalFrame.rows}, {0, 0, 0}, -1);
    cv::rectangle(finalFrame, {finalFrame.cols - 800 - 10, finalFrame.rows - lineHeight * 12 - 10},
                  {finalFrame.cols, finalFrame.rows}, {0, 0, 0},
                  -1);

    renderLine(finalFrame, "Run: " + std::to_string(run), 5, finalFrame.rows - 2 * lineHeight - 10);
    renderLine(finalFrame, "GREEN dots: Mapped objects", 5, finalFrame.rows - 1 * lineHeight - 10);
    renderLine(finalFrame, "RED dots: Unmapped objects", 5, finalFrame.rows - 0 * lineHeight - 10);

    std::stringstream ss;
    ss << estimator;
    std::string line;
    int i = 0;
    while (getline(ss, line)) {
        renderLine(finalFrame, line, finalFrame.cols - 800, finalFrame.rows - lineHeight * 11 + i++ * lineHeight - 10);
    }
}


#else //WITH_OPENCV

int main(int argc, char const *argv[]) {
    std::cout << "Please compile with OpenCV to evaluate." << std::endl;
    std::cout << "To compile with OpenCV add the flag -DWITH_OPENCV=ON to the cmake configure step." << std::endl;

    return 0;
}

#endif //WITH_OPENCV
