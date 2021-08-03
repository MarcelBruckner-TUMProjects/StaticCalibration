//
// Created by brucknem on 02.02.21.
//
#include <iostream>
#include "CMakeConfig.h"


#include "StaticCalibration/camera/RenderingPipeline.hpp"
#include "StaticCalibration/objects/WorldObject.hpp"
#include "StaticCalibration/objects/ObjectsLoading.hpp"
#include "StaticCalibration/CameraPoseEstimationBase.hpp"
#include "StaticCalibration/utils/CommandLineParser.hpp"
#include "CSVWriter.hpp"

#include "Eigen/Dense"
#include "glog/logging.h"

#ifdef WITH_OPENCV

#include <opencv2/opencv.hpp>
#include <StaticCalibration/CameraPoseEstimation.hpp>
#include <StaticCalibration/CameraPoseEstimationWithIntrinsics.hpp>

/**
 * Renders the current state of the estimator and some exemplary text onto the frame.
 *
 * @param finalFrame The frame to render to.
 * @param estimator The estimator that performs static calibration.
 * @param run The index of the current run.
 */
void
renderText(cv::Mat &finalFrame, const static_calibration::calibration::CameraPoseEstimationBase *estimator, int run);

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

#endif //WITH_OPENCV

/**
 * Initializes a writer to csv for the estimation results.
 *
 * @return The writer.
 */
static_calibration::evaluation::CSVWriter *initCSVWriters();

/**
 * Writes the values of the estimator to csv.
 *
 * @param csvWriter The csv writer.
 * @param run The current estimation run.
 * @param estimator The pose estimator->
 */
void writeToCSV(static_calibration::evaluation::CSVWriter *csvWriter, int run,
                static_calibration::calibration::CameraPoseEstimationBase *estimator);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** main */
int main(int argc, char const *argv[]) {
    srandom(time(nullptr));
    auto parsedOptions = static_calibration::utils::parseCommandLine(argc, argv);

    auto objects = static_calibration::calibration::loadObjects(parsedOptions.objectsFile, parsedOptions.pixelsFile);
    google::InitGoogleLogging("Static Calibration");

    static_calibration::calibration::CameraPoseEstimationBase *estimator;
    if (parsedOptions.withIntrinsics) {
        estimator = new static_calibration::calibration::CameraPoseEstimationWithIntrinsics(parsedOptions.intrinsics);
    } else {
        estimator = new static_calibration::calibration::CameraPoseEstimation(parsedOptions.intrinsics);
    }

    auto csvWriter = initCSVWriters();

#ifdef WITH_OPENCV
    cv::Mat evaluationFrame = cv::imread(parsedOptions.evaluationBackgroundFrame);
    evaluationFrame = addAlphaChannel(evaluationFrame);
    cv::Mat finalFrame;

    const char *windowName = "Evaluate Static Calibration";
    cv::namedWindow(windowName);
    int trackbarShowIds = 1;
    cv::createTrackbar("Show IDs", windowName, &trackbarShowIds, 1);
#endif //WITH_OPENCV

    Eigen::Vector3d translation;
    Eigen::Vector3d rotation;
    std::vector<double> intrinsics;

    int run = -1;
    int maxRuns = parsedOptions.evaluationRuns;

    for (int i = 0;; ++i) {
        if (estimator->isEstimationFinished()) {
            if (run >= 0) {
                writeToCSV(csvWriter, run, estimator);
            }
            if (run >= maxRuns) {
                break;
            }
            run++;
            estimator->clearWorldObjects();
            estimator->addWorldObjects(objects);
            estimator->setIntrinsics(parsedOptions.intrinsics);

#ifdef WITH_OPENCV
            estimator->estimateAsync(parsedOptions.logEstimationProgress);
#else //WITH_OPENCV
            estimator->estimate(parsedOptions.logEstimationProgress);
#endif //WITH_OPENCV
        }

#ifdef WITH_OPENCV
        translation = estimator->getTranslation();
        rotation = estimator->getRotation();
        intrinsics = estimator->getIntrinsics();

        finalFrame = evaluationFrame * 0.5;
        render(finalFrame, objects, translation, rotation, intrinsics, trackbarShowIds);
        renderText(finalFrame, estimator, run);

        cv::imshow(windowName, finalFrame);
        if ((char) cv::waitKey(1) == 'q') {
            break;
        }
#endif //WITH_OPENCV
    }

    return EXIT_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef WITH_OPENCV

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

    if (std::abs(vectorInCameraSpace.z()) > 1000) {
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


void
renderText(cv::Mat &finalFrame, const static_calibration::calibration::CameraPoseEstimationBase *estimator, int run) {
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

#endif //WITH_OPENCV

static_calibration::evaluation::CSVWriter *initCSVWriters() {
    auto csvWriter = new static_calibration::evaluation::CSVWriter(std::string("evaluation.csv"));

    *csvWriter << "Run"
               << "Correspondences"
               << "Valid Solution"
               << "Loss"
               << "Loss [Correspondences]"
               << "Loss [Lambdas]"
               << "Loss [Intrinsics]"
               << "Loss [Rotations]"
               << "Loss [Weights]"
               << "Translation [x]"
               << "Translation [y]"
               << "Translation [z]"
               << "Rotation [x]"
               << "Rotation [y]"
               << "Rotation [z]"
               << "Focal Length"
               << "Focal Length [Ratio]"
               << "Principal Point [u]"
               << "Principal Point [v]"
               << "Skew"
               << "Weights [Avg]"
               << "Weights [Min]"
               << "Weights [Max]"
               << static_calibration::evaluation::newline
               << static_calibration::evaluation::flush;
    return csvWriter;
}

void writeToCSV(static_calibration::evaluation::CSVWriter *csvWriter, int run,
                static_calibration::calibration::CameraPoseEstimationBase *estimator) {
    auto weights = estimator->getWeights();

    double min_w = 1e100;
    double max_w = -1e100;
    double sum_w = 0;
    for (const auto &weight : weights) {
        if (weight < min_w) {
            min_w = weight;
        }
        if (weight > max_w) {
            max_w = weight;
        }
        sum_w += weight;
    }

    *csvWriter << run
               << (int) weights.size()
               << estimator->hasFoundValidSolution()
               << estimator->getTotalLoss()
               << estimator->getCorrespondencesLoss()
               << estimator->getLambdasLoss()
               << estimator->getIntrinsicsLoss()
               << estimator->getRotationsLoss()
               << estimator->getWeightsLoss()
               << estimator->getTranslation()
               << estimator->getRotation()
               << estimator->getIntrinsics()
               << sum_w / weights.size()
               << min_w
               << max_w
               << static_calibration::evaluation::newline;
}