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
#include <StaticCalibration/utils/RenderUtils.hpp>
#include <boost/filesystem/path.hpp>


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
                     "For examples of config files see the 'config' folder."
                     "For explanations about the config files read the Readme.md or visit: \n"
                     "https://github.com/Brucknem/OpenDRIVE");

            return desc;
        }

        template<typename T>
        T getOrDefault(YAML::Node config, std::string variableName, T defaultValue) {
            if (config[variableName].IsDefined()) {
                return config[variableName].template as<T>();
            }
            return defaultValue;
        }

        std::string prefixFile(const std::string &basepath, const std::string &filename) {
            std::string result = filename;
            if (!filename.empty() && filename.at(0) != '/') {
                boost::filesystem::path basepathPath = basepath;
                result = basepathPath.parent_path().parent_path().string() + '/' + result;
            }
            return result;
        }

        template<typename T>
        T getOrThrow(YAML::Node config, std::string variableName) {
            try {
                return config[variableName].template as<T>();
            } catch (const YAML::Exception &e) {
                throw std::invalid_argument("Couldn't parse value of: " + variableName);
            }
        }

        ParsedOptions parseCommandLine(int argc, const char **argv) {
            auto desc = createOptionsDescription();

            std::string basepath = argv[0];
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

            std::vector<double> translation = getOrDefault(config, "translation", std::vector<double>{0, 0, 0});
            std::vector<double> rotation = getOrDefault(config, "rotation", std::vector<double>{0, 0, 0});
            if (config["ros_tf2_coordinates"].IsDefined() && config["ros_tf2_coordinates"].as<bool>()) {
                translation = static_calibration::utils::translationToROStf2(translation, true);
                rotation = static_calibration::utils::rotationToROStf2(rotation, true);
            }

            ParsedOptions parsedOptions;
            parsedOptions = ParsedOptions{
                    prefixFile(basepath, getOrThrow<std::string>(config, "output_dir")),
                    prefixFile(basepath, getOrThrow<std::string>(config, "objects_file")),
                    prefixFile(basepath, getOrThrow<std::string>(config, "pixels_file")),
                    prefixFile(basepath, getOrThrow<std::string>(config, "mapping_file")),
                    prefixFile(basepath, getOrDefault(config, "lane_samples_file", std::string())),
                    prefixFile(basepath, getOrDefault(config, "explicit_road_marks_file", std::string())),
                    getOrThrow<std::string>(config, "measurement_point"),
                    getOrThrow<std::string>(config, "camera_name"),
                    prefixFile(basepath, getOrDefault(config, "background_frame", std::string())),
                    getOrDefault(config, "evaluation_runs", 10),
                    getOrThrow<std::vector<double>>(config, "intrinsics"),
                    translation,
                    rotation,
                    getOrDefault(config, "optimize_intrinsics", false),
                    getOrDefault(config, "log_optimization", true),
                    getOrDefault(config, "max_pixel_distance_for_mapping", 1000),
                    getOrDefault(config, "max_matches_per_image_object", 5),
                    getOrDefault(config, "max_new_elements_per_mapping", -1),
            };

            return parsedOptions;
        }
    }
}
