//
// Created by brucknem on 02.02.21.
//
#include <iostream>
#include "CMakeConfig.h"


#include "StaticCalibration/camera/RenderingPipeline.hpp"
#include "StaticCalibration/objects/WorldObject.hpp"
#include "StaticCalibration/objects/DataSet.hpp"
#include "StaticCalibration/CameraPoseEstimationBase.hpp"
#include "StaticCalibration/utils/CommandLineParser.hpp"
#include "CSVWriter.hpp"

#include "Eigen/Dense"
#include "glog/logging.h"

#ifdef WITH_OPENCV

#include <opencv2/opencv.hpp>
#include <StaticCalibration/CameraPoseEstimation.hpp>
#include <StaticCalibration/CameraPoseEstimationWithIntrinsics.hpp>
#include "StaticCalibration/utils/RenderUtils.hpp"

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

    auto dataSet = static_calibration::objects::DataSet(parsedOptions.objectsFile, parsedOptions.pixelsFile,
                                                        parsedOptions.mappingFile);

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
    evaluationFrame = static_calibration::utils::addAlphaChannel(evaluationFrame);
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
            estimator->setDataSet(dataSet);
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
        static_calibration::utils::render(finalFrame, dataSet.getWorldObjects(), translation, rotation, intrinsics,
                                          trackbarShowIds);
        static_calibration::utils::renderText(finalFrame, estimator, run);

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