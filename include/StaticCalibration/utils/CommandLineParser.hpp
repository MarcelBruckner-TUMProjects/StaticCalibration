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
             * The path to the objects file containing the world objects.
             */
            std::string objectsFile;

            /**
             * The path to the pixels file containing the pixels corresponding to world objects.
             */
            std::string pixelsFile;

            /**
             * The path to the file to which the resulting calibration parameters are written.
             */
            std::string calibrationParamsFile;

            /**
             * The path to the background image rendered during evaluation.
             */
            std::string evaluationBackgroundFrame;

            /**
             * The number of evaluation runs.
             */
            int evaluationRuns;

            /**
             * The size in pixels of the camera frame output.
             */
            Eigen::Vector2i imageSize;

            /**
             * The focal length in pixels of the camera according to the pinhole camera model.
             */
            double focalLength;

            /**
             * The ratio of the focal length of the X axis to the focal length of the Y axis of the camera according to the pinhole camera model.
             */
            double focalLengthRatio;

            /**
             * The principal point in pixels of the camera according to the pinhole camera model.
             */
            Eigen::Vector2d principalPoint;

            /**
             * The skew of the camera according to the pinhole camera model.
             */
            double skew;

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
         * Parses a string containing a comma separated tuple of values.
         * Prints a help text and stops execution if the tuple does not consist of exactly 2 numeric values.
         *
         * @param input The string to parse.
         *
         * @return The parsed tuple.
         */
        Eigen::Vector2d parseVectorValue(const std::string &input);

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
