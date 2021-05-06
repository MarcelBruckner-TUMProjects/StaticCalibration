//
// Created by brucknem on 02.02.21.
//

#include <thread>
#include "StaticCalibration/objects/ObjectsLoading.hpp"
#include "StaticCalibration/CameraPoseEstimation.hpp"

#include "glog/logging.h"

#include "CommandLineParser.hpp"

int main(int argc, char const *argv[]) {
    srandom(time(nullptr));
    auto parsedOptions = static_calibration::app::parseCommandLine(argc, argv);

    auto objects = static_calibration::calibration::loadObjects(parsedOptions.objectsFile, parsedOptions.pixelsFile,
                                                                parsedOptions.imageSize);
    static_calibration::calibration::CameraPoseEstimation estimator;
    estimator.addWorldObjects(objects);
    estimator.guessIntrinsics(parsedOptions.focalLength, parsedOptions.focalLengthRatio, parsedOptions.principalPoint,
                              parsedOptions.skew);

    google::InitGoogleLogging("Static Calibration");
    estimator.estimate(parsedOptions.logEstimationProgress);

    return EXIT_SUCCESS;
}
