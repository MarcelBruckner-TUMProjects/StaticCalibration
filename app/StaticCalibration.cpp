//
// Created by brucknem on 02.02.21.
//
#include <iostream>
#include <random>
#include "CMakeConfig.h"


#include "StaticCalibration/camera/RenderingPipeline.hpp"
#include "StaticCalibration/objects/WorldObject.hpp"
#include "StaticCalibration/objects/DataSet.hpp"
#include "StaticCalibration/CameraPoseEstimationBase.hpp"
#include "StaticCalibration/utils/CommandLineParser.hpp"
#include "StaticCalibration/utils/CSVWriter.hpp"

#include "Eigen/Dense"
#include "glog/logging.h"

#ifdef WITH_OPENCV

#include <opencv2/opencv.hpp>
#include <StaticCalibration/CameraPoseEstimation.hpp>
#include <StaticCalibration/CameraPoseEstimationWithIntrinsics.hpp>
#include <StaticCalibration/utils/Formatters.hpp>
#include "StaticCalibration/utils/RenderUtils.hpp"

#endif //WITH_OPENCV

/**
 * Initializes a writer to csv for the estimation results.
 *
 * @return The writer.
 */
static_calibration::evaluation::CSVWriter *initCSVWriters(const std::string &base_path);

/**
 * Writes the values of the estimator to csv.
 *
 * @param csvWriter The csv writer.
 * @param run The current estimation run.
 * @param estimator The pose estimator
 */
void writeToCSV(static_calibration::evaluation::CSVWriter *csvWriter, int run,
                static_calibration::calibration::CameraPoseEstimationBase *estimator, double evaluationError);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** main */
int main(int argc, char const *argv[]) {
    srandom(time(nullptr));
    auto parsedOptions = static_calibration::utils::parseCommandLine(argc, argv);
    auto basename = boost::filesystem::path(parsedOptions.outputDir);

    auto dataSet = static_calibration::objects::DataSet(parsedOptions.objectsFile,
                                                        parsedOptions.explicitRoadMarksFile,
                                                        parsedOptions.pixelsFile,
                                                        parsedOptions.mappingFile);
    google::InitGoogleLogging("Static Calibration");

    static_calibration::calibration::CameraPoseEstimationBase *estimator;
    if (parsedOptions.withIntrinsics) {
        estimator = new static_calibration::calibration::CameraPoseEstimationWithIntrinsics(parsedOptions.intrinsics);
    } else {
        estimator = new static_calibration::calibration::CameraPoseEstimation(parsedOptions.intrinsics);
    }


#ifdef WITH_OPENCV
    cv::Mat evaluationFrame = cv::imread(parsedOptions.evaluationBackgroundFrame);
    evaluationFrame = static_calibration::utils::addAlphaChannel(evaluationFrame);
    cv::Mat finalFrame;

    const char *windowName = "Evaluate Static Calibration";
    cv::namedWindow(windowName);
    int trackbarShowIds = 1;
    int maxRenderDistance = 600;
    cv::createTrackbar("Show IDs", windowName, &trackbarShowIds, 1);
    cv::createTrackbar("Render Distance", windowName, &maxRenderDistance, 2000);
#endif //WITH_OPENCV

    Eigen::Vector3d translation(parsedOptions.translation.data());
    Eigen::Vector3d rotation(parsedOptions.rotation.data());
    std::vector<double> intrinsics = parsedOptions.intrinsics;

    Eigen::Vector3d initialTranslation = translation;
    Eigen::Vector3d initialRotation = rotation;
    std::vector<double> initialIntrinsics = intrinsics;

    bool mappingsCreated = false;
    std::vector<std::map<std::string, std::string>> mappings;

    int run = -1;
    int numMappings = 0;
    int maxRuns = parsedOptions.evaluationRuns;
    double evaluationError = -1;

    while (run < maxRuns) {
        if (estimator->isEstimationFinished()) {
            if (run >= 0) {
                auto outDir = basename / std::to_string(evaluationError);
                auto baseOutDir = outDir;
                for (int fileExtension = 0;; ++fileExtension) {
                    if (!boost::filesystem::exists(outDir)) {
                        break;
                    }
                    outDir = boost::filesystem::path(baseOutDir.string() + "_" + std::to_string(fileExtension));
                }
                boost::filesystem::create_directories(outDir);
                auto csvWriter = initCSVWriters(outDir.string());
                writeToCSV(csvWriter, run, estimator, evaluationError);

                std::ofstream outFile;

                outFile.open((outDir / "transformations.launch").string());
                auto rosXML = static_calibration::utils::toROStf2Node(*estimator, parsedOptions.measurementPointName,
                                                                      parsedOptions.cameraName);
                std::cout << rosXML << std::endl;
                outFile << rosXML;
                outFile.close();

                outFile.open((outDir / "intrinsics.yaml").string());
                auto intrinsicsYAML = static_calibration::utils::toROSParamsIntrinsics(*estimator,
                                                                                       parsedOptions.measurementPointName,
                                                                                       parsedOptions.cameraName);
                std::cout << intrinsicsYAML << std::endl;
                outFile << intrinsicsYAML;
                outFile.close();

                outFile.open((outDir / "mapping.yaml").string());
                auto mappingYAML = static_calibration::utils::mergedMappingToYAML(dataSet);
                std::cout << mappingYAML << std::endl;
                outFile << mappingYAML;
                outFile.close();

#ifdef WITH_OPENCV
                cv::Mat outFrame = evaluationFrame * 0.5;
                static_calibration::utils::render(outFrame, dataSet, translation, rotation, intrinsics, true,
                                                  maxRenderDistance);
                cv::imwrite((outDir / "with_ids.png").string(),
                            static_calibration::utils::removeAlphaChannel(outFrame));
                outFrame = evaluationFrame * 0.5;
                static_calibration::utils::render(outFrame, dataSet, translation, rotation, intrinsics, false,
                                                  maxRenderDistance);
                cv::imwrite((outDir / "without_ids.png").string(),
                            static_calibration::utils::removeAlphaChannel(outFrame));
#endif //WITH_OPENCV

                if (mappings.empty()) {
                    if (numMappings > 0) {
                        break;
                    }
                    // After first run only with fixed mapping, set the new initial extrinsics to the found ones.
                    initialTranslation = translation;
                    initialRotation = rotation;
                    initialIntrinsics = intrinsics;
                    mappings = dataSet.createAllMappings(translation, rotation, intrinsics,
                                                         parsedOptions.maxPixelDistanceForMapping,
                                                         parsedOptions.maxMatchesPerImageObject,
                                                         parsedOptions.maxNewElementsPerMapping);
                    numMappings = mappings.size();
                }
                dataSet.setMappingExtension(mappings[mappings.size() - 1]);
                mappings.pop_back();
            }

            run++;
            estimator->setDataSet(dataSet);
            estimator->guessTranslation(initialTranslation);
            estimator->guessRotation(initialRotation);
            estimator->setIntrinsics(initialIntrinsics);

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
        static_calibration::utils::render(finalFrame, dataSet, translation, rotation, intrinsics, trackbarShowIds,
                                          maxRenderDistance);
        evaluationError = dataSet.evaluate(translation, rotation, intrinsics);
        static_calibration::utils::renderText(finalFrame, estimator, run, numMappings, evaluationError);

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

static_calibration::evaluation::CSVWriter *initCSVWriters(const std::string &base_path) {
    auto csvWriter = new static_calibration::evaluation::CSVWriter(
            (boost::filesystem::path(base_path) / "evaluation.csv").string());

    *csvWriter << "Run"
               << "Correspondences"
               << "Valid Solution"
               << "Evaluation Error"
               << "Loss"
               << "Loss [Correspondences]"
               << "Loss [Road Marks]"
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
                static_calibration::calibration::CameraPoseEstimationBase *estimator, double evaluationError) {
    auto weights = estimator->getWeights();

    double min_w = 1e100;
    double max_w = -1e100;
    double sum_w = 0;
    for (const auto &weight: weights) {
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
               << evaluationError
               << estimator->getTotalLoss()
               << estimator->getCorrespondencesLoss()
               << estimator->getExplicitRoadMarksLoss()
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