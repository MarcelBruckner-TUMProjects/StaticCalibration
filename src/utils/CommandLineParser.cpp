//
// Created by brucknem on 02.02.21.
//

#include "StaticCalibration/utils/CommandLineParser.hpp"
#include "Eigen/Dense"

#include <iostream>

#include <boost/algorithm/string/split.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/program_options.hpp>


namespace static_calibration {
    namespace app {

#define OBJECTS_FILE_OPTION_NAME std::string("objects_file")
#define PIXELS_FILE_OPTION_NAME std::string("pixels_file")
#define FOCAL_LENGTH_OPTION_NAME std::string("focal_length")
#define FOCAL_LENGTH_RATIO_OPTION_NAME std::string("focal_length_ratio")
#define IMAGE_SIZE_OPTION_NAME std::string("image_size")
#define PRINCIPAL_POINT_OPTION_NAME std::string("principal_point")
#define SKEW_OPTION_NAME std::string("skew")
#define LOG_ESTIMATION_PROGRESS_OPTION_NAME std::string("log")

        bool logEstimationProgress = false;

        void checkCommandLineOptionPresent(const boost::program_options::variables_map &variables_map,
                                           const std::string &name) {
            if (variables_map.count(name) <= 0) {
                std::cout << "Please provide some " << name << ". See the help for more information about the usage."
                          << std::endl;
                exit(EXIT_FAILURE);
            }
        }

        boost::program_options::options_description createOptionsDescription() {
            boost::program_options::options_description desc("Usage of the Static Calibration estimator", 120);

            desc.add_options()
                    ("help,h", "Produce this help message.");

            desc.add_options()
                    ((OBJECTS_FILE_OPTION_NAME + ",o").c_str(), boost::program_options::value<std::string>(),
                     "The path to the file including the world objects. The path can be relative or absolute. "
                     "To create the objects file from an OpenDRIVE HD map visit: \n"
                     "https://github.com/Brucknem/OpenDRIVE");

            desc.add_options()
                    ((PIXELS_FILE_OPTION_NAME + ",p").c_str(), boost::program_options::value<std::string>(),
                     "The path to the file including the marked pixels. The path can be relative or absolute. "
                     "To create the pixels file from a keyframe extracted from a video visit: \n"
                     "https://github.com/Brucknem/DataAnnotationTools");

            desc.add_options()
                    ((IMAGE_SIZE_OPTION_NAME + ",i").c_str(), boost::program_options::value<std::string>(),
                     "The size of the image.\n"
                     "OpenCV flips images when rendering along the X axis. "
                     "We thus need the image size to flip the pixels during estimation.");

            desc.add_options()
                    ((FOCAL_LENGTH_OPTION_NAME + ",f").c_str(), boost::program_options::value<double>(),
                     "The focal length of the camera in pixels. "
                     "To get information about the used pinhole camera model please visit: \nhttps://en.wikipedia.org/wiki/Pinhole_camera_model");

            desc.add_options()
                    ((FOCAL_LENGTH_RATIO_OPTION_NAME + ",r").c_str(),
                     boost::program_options::value<double>()->default_value(1),
                     "The ratio of the focal length of the X axis to the focal length of the Y axis of the camera."
                     "To get information about the used pinhole camera model please visit: \nhttps://en.wikipedia.org/wiki/Pinhole_camera_model");

            desc.add_options()
                    ((PRINCIPAL_POINT_OPTION_NAME + ",u").c_str(), boost::program_options::value<std::string>(),
                     "The principal point of the camera in pixels. "
                     "Please provide the values as a comma separated tuple of: \"principal_X, principal_Y\".\n"
                     "To get information about the used pinhole camera model please visit: \nhttps://en.wikipedia.org/wiki/Pinhole_camera_model");

            desc.add_options()
                    ((SKEW_OPTION_NAME + ",s").c_str(), boost::program_options::value<double>()->default_value(1),
                     "The skew of the camera.\n"
                     "To get information about the used pinhole camera model please visit: \nhttps://en.wikipedia.org/wiki/Pinhole_camera_model");

            desc.add_options()
                    ((LOG_ESTIMATION_PROGRESS_OPTION_NAME + ",l").c_str(),
                     boost::program_options::bool_switch(&logEstimationProgress),
                     "Flag if the progress of the estimation should be logged.");
            return desc;
        }

        Eigen::Vector2d parseVectorValue(const std::string &input) {
            std::vector<std::string> parserBuffer;
            std::vector<double> parsedValuesBuffer;
            boost::split(parserBuffer, input, [](char c) { return c == ','; });
            if (parserBuffer.size() != 2) {
                std::cout << "Provide intrinsics as comma separated tuple. "
                             "See the help for more information about the usage.";
                exit(EXIT_FAILURE);
            }
            BOOST_FOREACH(std::string value, parserBuffer) {
                            parsedValuesBuffer.emplace_back(boost::lexical_cast<double>(boost::trim_copy(value)));
                        }
            return Eigen::Vector2d{parsedValuesBuffer[0], parsedValuesBuffer[1]};
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
            checkCommandLineOptionPresent(variables_map, OBJECTS_FILE_OPTION_NAME);
            checkCommandLineOptionPresent(variables_map, PIXELS_FILE_OPTION_NAME);
            checkCommandLineOptionPresent(variables_map, IMAGE_SIZE_OPTION_NAME);
            checkCommandLineOptionPresent(variables_map, FOCAL_LENGTH_OPTION_NAME);
            checkCommandLineOptionPresent(variables_map, PRINCIPAL_POINT_OPTION_NAME);

            ParsedOptions parsedOptions{
                    variables_map[OBJECTS_FILE_OPTION_NAME].as<std::string>(),
                    variables_map[PIXELS_FILE_OPTION_NAME].as<std::string>(),
                    parseVectorValue(variables_map[IMAGE_SIZE_OPTION_NAME].as<std::string>()).cast<int>(),
                    variables_map[FOCAL_LENGTH_OPTION_NAME].as<double>(),
                    variables_map[FOCAL_LENGTH_RATIO_OPTION_NAME].as<double>(),
                    parseVectorValue(variables_map[PRINCIPAL_POINT_OPTION_NAME].as<std::string>()),
                    variables_map[SKEW_OPTION_NAME].as<double>(),
                    logEstimationProgress
            };

            return parsedOptions;
        }
    }
}
