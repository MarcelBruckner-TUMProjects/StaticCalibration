//
// Created by brucknem on 02.02.21.
//

#ifndef STATICCALIBRATION_COMMANDLINEPARSER_HPP
#define STATICCALIBRATION_COMMANDLINEPARSER_HPP

#include "CMakeConfig.h"

#include "Eigen/Dense"
#include <boost/algorithm/string/split.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/program_options.hpp>

namespace static_calibration {
    namespace utils {

        /**
         * Parser result wrapping the options for further usage.
         */
        struct ParsedOptions {

            /**
             * The path to the output directory.
             */
            std::string outputDir;

            /**
             * The path to the objects file containing the world objects.
             */
            std::string objectsFile;

            /**
             * The path to the file containing the pixels corresponding to world objects.
             */
            std::string pixelsFile;

            /**
             * The path to the file containing the mapping between 3D world objects and 2D image objects.
             */
            std::string mappingFile;

            /**
             * The path to the file containing the lane samples.
             */
            std::string laneSamplesFile;

            /**
             * The path to the file containing the explicit road marks.
             */
            std::string explicitRoadMarksFile;

            /**
             * The name of the measurement point, e.g. s50.
             */
            std::string measurementPointName;

            /**
             * The name of the camera at the measurement point, e.g. s_cam_near.
             */
            std::string cameraName;

            /**
             * The path to the background image rendered during evaluation.
             */
            std::string evaluationBackgroundFrame;

            /**
             * The number of evaluation runs.
             */
            int evaluationRuns;

            /**
             * The intrinsic parameters of the camera according to the pinhole camera model.
             */
            std::vector<double> intrinsics;

            std::vector<double> translation;
            std::vector<double> rotation;

            /**
             * Flag if intrinsics should be optimized.
             */
            bool withIntrinsics;

            /**
             * Flag if the progress of the ceres optimizer should be logged.
             */
            bool logEstimationProgress;
        };

        /**
         * Creates the command line options description wrapper.
         *
         * @return The command line options description wrapper.
         */
        boost::program_options::options_description createOptionsDescription();

        /**
         * Parses the command line options.
         * Prints help messages and stops execution if the parameters are in wrong formats.
         *
         * @param argc The number of command line arguments.
         * @param argv The command line arguments.
         *
         * @return The parsed options wrapped in a usable format.
         */
        ParsedOptions parseCommandLine(int argc, char const *argv[]);
    }
}

#endif //STATICCALIBRATION_COMMANDLINEPARSER_HPP
