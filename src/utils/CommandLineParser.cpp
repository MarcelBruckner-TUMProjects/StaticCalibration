//
// Created by brucknem on 02.02.21.
//

#include "CMakeConfig.h"

#include "StaticCalibration/utils/CommandLineParser.hpp"
#include "Eigen/Dense"

#include <iostream>

#include <boost/algorithm/string/split.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/program_options.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <fstream>
#include <yaml-cpp/yaml.h>


namespace static_calibration {
    namespace utils {

        const char *CONFIG_FILE_OPTION_NAME = "config";

        boost::program_options::options_description createOptionsDescription() {
            boost::program_options::options_description desc("Usage of the Static Calibration estimator", 120);

            desc.add_options()
                    ("help,h", "Produce this help message.");

            desc.add_options()
                    ((std::string(CONFIG_FILE_OPTION_NAME) + ",c").c_str(),
                     boost::program_options::value<std::string>(),
                     "The path to the config file."
                     "For explanations about the config files see the 'config' folder or visit: \n"
                     "https://github.com/Brucknem/OpenDRIVE");

            return desc;
        }

        ParsedOptions parseCommandLine(int argc, const char **argv) {
            auto desc = createOptionsDescription();

            boost::program_options::variables_map variables_map;
            boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), variables_map);
            boost::program_options::notify(variables_map);

            if (variables_map.count("help") > 0) {
                std::cout << desc << std::endl;
                exit(EXIT_SUCCESS);
            }

            std::string configFileName = variables_map["config"].as<std::string>();
            if (boost::starts_with(configFileName, "./")) {
                configFileName = configFileName.substr(2, configFileName.length());
            }
            if (!boost::starts_with(configFileName, "/")) {
                configFileName = "../" + configFileName;
            }
            YAML::Node config = YAML::LoadFile(configFileName);

            std::string evaluationBackgroundFrame;
#ifdef WITH_OPENCV
            evaluationBackgroundFrame = config["background_frame"].as<std::string>();
#endif //WITH_OPENCV

            int evaluationRuns = 10;
            if (config["evaluation_runs"].IsDefined()) {
                evaluationRuns = config["evaluation_runs"].as<int>();
            }

            auto objectsFile = config["objects_file"].as<std::string>();
            auto pixelsFile = config["pixels_file"].as<std::string>();
            auto measurementPoint = config["measurement_point"].as<std::string>();
            auto cameraName = config["camera_name"].as<std::string>();
            auto intrinsics = config["intrinsics"].as<std::vector<double>>();

            auto optimizeIntrinsics = false;
            if (config["optimize_intrinsics"].IsDefined()) {
                optimizeIntrinsics = config["optimize_intrinsics"].as<bool>();
            }

            auto logOptimization = true;
            if (config["log_optimization"].IsDefined()) {
                logOptimization = config["log_optimization"].as<bool>();
            }

            ParsedOptions parsedOptions{
                    objectsFile,
                    pixelsFile,
                    measurementPoint,
                    cameraName,
                    evaluationBackgroundFrame,
                    evaluationRuns,
                    intrinsics,
                    optimizeIntrinsics,
                    logOptimization
            };

            return parsedOptions;
        }
    }
}
