//
// Created by brucknem on 02.02.21.
//

#include <thread>
#include "StaticCalibration/objects/ObjectsLoading.hpp"
#include "StaticCalibration/CameraPoseEstimation.hpp"

#include "glog/logging.h"
#include "yaml-cpp/yaml.h"
#include <boost/filesystem.hpp>

#include "StaticCalibration/utils/CommandLineParser.hpp"

int main(int argc, char const *argv[]) {
    srandom(time(nullptr));
    auto parsedOptions = static_calibration::utils::parseCommandLine(argc, argv);

    auto objects = static_calibration::calibration::loadObjects(parsedOptions.objectsFile, parsedOptions.pixelsFile,
                                                                parsedOptions.imageSize);
    static_calibration::calibration::CameraPoseEstimation estimator;
    estimator.addWorldObjects(objects);
    estimator.guessIntrinsics(parsedOptions.focalLength, parsedOptions.focalLengthRatio, parsedOptions.principalPoint,
                              parsedOptions.skew);

    google::InitGoogleLogging("Static Calibration");
    estimator.estimate(parsedOptions.logEstimationProgress);

    if (!estimator.hasFoundValidSolution()) {
        std::cout << "Couldn't find valid solution! "
                     "This may happen due to initializations. "
                     "If it persists after rerunning please check if the input data is correct." << std::endl;
        exit(EXIT_FAILURE);
    }

    auto outFilename = boost::filesystem::path(parsedOptions.calibrationParamsFile);
    auto outPath = outFilename.parent_path();
    if (!outPath.empty() && (!boost::filesystem::exists(outPath) && !boost::filesystem::create_directories(outPath))) {
        std::cout << "Could not create the path to the calibration params file." << std::endl;
        return EXIT_SUCCESS;
    }

    if (outFilename.extension() != ".yaml") {
        outFilename += ".yaml";
    }

    std::ofstream outFile;
    outFile.open(outFilename.string());
    outFile << estimator.toYAML();
    outFile.close();

    return EXIT_SUCCESS;
}
