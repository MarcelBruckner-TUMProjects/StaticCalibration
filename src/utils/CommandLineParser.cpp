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
    namespace utils {

        const char *OBJECTS_FILE_OPTION_NAME = "objects_file";
        const char *PIXELS_FILE_OPTION_NAME = "pixels_file";
        const char *EVALUATION_BACKGROUND_FRAME_OPTION_NAME = "evaluation_background_frame";
        const char *EVALUATION_RUNS_OPTION_NAME = "evaluation_runs";
        const char *INTRINSICS_OPTION_NAME = "intrinsics";
        const char *WITH_INTRINSICS_OPTION_NAME = "with_intrinsics";
        const char *LOG_ESTIMATION_PROGRESS_OPTION_NAME = "log";
        const char *CAMERA_NAME_OPTION_NAME = "camera_name";
        const char *MEASUREMENT_POINT_OPTION_NAME = "measurement_point";

        bool logEstimationProgress = false;
        bool withIntrinsics = false;

        boost::program_options::options_description createOptionsDescription() {
            boost::program_options::options_description desc("Usage of the Static Calibration estimator", 120);

            desc.add_options()
                    ("help,h", "Produce this help message.");

            desc.add_options()
                    ((std::string(OBJECTS_FILE_OPTION_NAME) + ",o").c_str(),
                     boost::program_options::value<std::string>()->default_value("../misc/objects.yaml"),
                     "The path to the file including the world objects. The path can be relative or absolute. "
                     "The objects need to be in the right-handed coordinate system with the X-axis pointing east, the Y-axis pointing north and the Z-axis giving the height. "
                     "To create the objects file from an OpenDRIVE HD map visit: \n"
                     "https://github.com/Brucknem/OpenDRIVE");

            desc.add_options()
                    ((std::string(PIXELS_FILE_OPTION_NAME) + ",p").c_str(),
                     boost::program_options::value<std::string>()->default_value("../misc/pixels.yaml"),
                     "The path to the file including the marked pixels. The path can be relative or absolute. "
                     "To create the pixels file from a keyframe extracted from a video visit: \n"
                     "https://github.com/Brucknem/DataAnnotationTools");

            desc.add_options()
                    ((std::string(MEASUREMENT_POINT_OPTION_NAME) + ",m").c_str(),
                     boost::program_options::value<std::string>()->default_value("s40"),
                     "The name of the measurement point (the gantry bridge)");

            desc.add_options()
                    ((std::string(CAMERA_NAME_OPTION_NAME) + ",c").c_str(),
                     boost::program_options::value<std::string>()->default_value("s_cam_far"),
                     "The name of the camera at the given measurement point.");

#ifdef WITH_OPENCV
            desc.add_options()
                    ((std::string(EVALUATION_BACKGROUND_FRAME_OPTION_NAME) + ",e").c_str(),
                     boost::program_options::value<std::string>()->default_value("../misc/test_frame.png"),
                     "The path to the file containing the background frame used for rendering during evaluation."
                     " The path can be relative or absolute."
                    );
#endif //WITH_OPENCV

#ifndef WITH_COVERAGE
            desc.add_options()
                    ((std::string(EVALUATION_RUNS_OPTION_NAME) + ",n").c_str(),
                     boost::program_options::value<int>()->default_value(1000000),
                     "The number of runs performed during evaluation.");
#endif //WITH_COVERAGE

            desc.add_options()
                    ((std::string(INTRINSICS_OPTION_NAME) + ",i").c_str(),
                     boost::program_options::value<std::vector<double>>()->multitoken(),
                     "The intrinsic parameters of the pinhole camera model. "
                     "Provide the parameters as space separated list of values: focal_x focal_y principal_x principal_y [skew (optional, defaults to 0)]. "
                     "All parameters need to be in pixels (except for skew). "
                     "To get information about the used pinhole camera model please visit: \nhttps://en.wikipedia.org/wiki/Pinhole_camera_model");

            desc.add_options()
                    ((std::string(WITH_INTRINSICS_OPTION_NAME) + ",w").c_str(),
                     boost::program_options::bool_switch(&withIntrinsics),
                     "Flag if the intrinsics should be optimized.");

            desc.add_options()
                    ((std::string(LOG_ESTIMATION_PROGRESS_OPTION_NAME) + ",l").c_str(),
                     boost::program_options::bool_switch(&logEstimationProgress),
                     "Flag if the progress of the estimation should be logged to STDOUT.");
            return desc;
        }

        ParsedOptions parseCommandLine(int argc, const char **argv) {
            auto desc = createOptionsDescription();

            boost::program_options::variables_map variables_map;
            boost::program_options::store(
                    boost::program_options::command_line_parser(argc, argv).options(desc).allow_unregistered().run(),
                    variables_map
            );
            boost::program_options::notify(variables_map);

            if (variables_map.count("help") > 0) {
                std::cout << desc << std::endl;
                exit(EXIT_SUCCESS);
            }

            std::string evaluationBackgroundFrame;
#ifdef WITH_OPENCV
            evaluationBackgroundFrame = variables_map[EVALUATION_BACKGROUND_FRAME_OPTION_NAME].as<std::string>();
#endif //WITH_OPENCV

            int evaluationRuns = 1;
#ifndef WITH_COVERAGE
            evaluationRuns = variables_map[EVALUATION_RUNS_OPTION_NAME].as<int>();
#endif //WITH_COVERAGE

            ParsedOptions parsedOptions{
                    variables_map[OBJECTS_FILE_OPTION_NAME].as<std::string>(),
                    variables_map[PIXELS_FILE_OPTION_NAME].as<std::string>(),
                    variables_map[MEASUREMENT_POINT_OPTION_NAME].as<std::string>(),
                    variables_map[CAMERA_NAME_OPTION_NAME].as<std::string>(),
                    evaluationBackgroundFrame,
                    evaluationRuns,
                    variables_map[INTRINSICS_OPTION_NAME].as<std::vector<double>>(),
                    withIntrinsics,
                    logEstimationProgress
            };

            return parsedOptions;
        }
    }
}
