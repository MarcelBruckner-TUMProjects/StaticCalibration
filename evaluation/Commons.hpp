//
// Created by brucknem on 13.01.21.
//

#ifndef CAMERASTABILIZATION_COMMONS_HPP
#define CAMERASTABILIZATION_COMMONS_HPP

#include "CMakeConfig.h"

#ifdef WITH_OPENCV
#include <stdexcept>
#include "StaticCalibration/camera/RenderingPipeline.hpp"
#include "opencv2/opencv.hpp"
#include "Eigen/Dense"
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/program_options.hpp>

#include "boost/date_time/gregorian/gregorian.hpp"
#include "boost/date_time/posix_time/posix_time.hpp"

namespace po = boost::program_options;

namespace static_calibration {
    namespace evaluation {

        boost::posix_time::ptime getNow();

        std::string getNowSuffix();

        /**
         * Gets the default video file path.
         */
        std::string getDefaultVideoFile();

        /**
         * Opens the given path and loads it as a video capture.
         *
         * @param videoFileName The full file name of the video.
         * @return A video capture with the loaded video.
         * @throws std::invalid_argument if video not loaded
         */
        cv::VideoCapture openVideoCapture(const std::string &videoFileName);

        /**
         * Generates a readable string containing the message, duration and fps.
         *
         * @param message The message prefix.
         * @param milliseconds The duration in milliseconds.
         * @return A formatted string with the duration.
         */
        std::string durationInfo(const std::string &message, long milliseconds);

        /**
         * Adds text to a frame.
         *
         * @param frame The frame to add text to.
         * @param text The text to add.
         * @param x The left starting expectedPixel location.
         * @param y The upper starting expectedPixel location.
         */
        cv::Mat addText(const cv::Mat &frame, const std::string &text, double fontSize, int x, int y, cv::Scalar
        color = cv::Scalar(255, 255, 0));

        /**
         * Adds runtime info to the frame.
         *
         * @param frame The frame to add message to.
         * @param message The message to add.
         * @param milliseconds The duration in milliseconds.
         * @param x The left starting expectedPixel location.
         * @param y The upper starting expectedPixel location.
         */
        cv::Mat
        addRuntimeToFrame(const cv::Mat &_frame, const std::string &message, long milliseconds, double fontSize, int x,
                          int y);

        /**
         * Removes the padding pixels from the frame.
         *
         * @param frame The frame to remove the pixels from.
         * @param padding The number of pixels to remove from each side.
         * @return The padded frame.
         */
        cv::Mat pad(const cv::Mat &frame, int padding);

        /**
         * @get a random double [0, 1)
         */
        double getRandom01();

        /**
         * Concatenates the given writeFrames into a vector.
         */
        template<typename T>
        std::vector<cv::Mat> concatenate(const std::initializer_list<T> &frames, int padding = 0, cv::Size size =
        cv::Size());

        template<typename T>
        cv::Mat vconcat(const std::initializer_list<T> &frames, int padding = 0, cv::Size size = cv::Size());

        template<typename T>
        cv::Mat hconcat(const std::initializer_list<T> &frames, int padding = 0, cv::Size size = cv::Size());

        cv::Mat MatOfSize(cv::Size size, int type = CV_8UC3);

        cv::cuda::GpuMat GpuMatOfSize(cv::Size size, int type = CV_8UC3);

        cv::cuda::GpuMat cvtColor(cv::cuda::GpuMat frame, int colorSpace = cv::COLOR_BGR2GRAY);

        /**
         * Base class for all evaluation setups that run on a video.
         * Wraps the main loop and field initializations.
         */
        class ImageSetup {
        protected:
            /**
             * The current CPU frame.
             */
            cv::Mat frameCPU;
            /**
             * The final CPU frame after processing. This will be rendered at the end.
             */
            cv::Mat finalFrame;

            /**
             * The current GPU frame.
             */
            cv::cuda::GpuMat frameGPU;

            /**
             * A CPU frame buffer for additional calculations.
             */
            cv::Mat bufferCPU;

            /**
             * A GPU frame buffer for additional calculations.
             */
            cv::cuda::GpuMat bufferGPU;

            /**
             * The file name of the video.
             */
            std::string inputResource;

            /**
             * The title name of the window.
             */
            std::string windowName;

            /**
             * Pressed pressedKey.
             */
            int pressedKey = -1;

            /**
             * A scaling factor applied to the frame before calculation.
             */
            double calculationScaleFactor = 1;

            /**
             * A scaling factor applied to the final frame before rendering.
             */
            double renderingScaleFactor = 0.5;

            /**
             * The total duration of the algorithms.
             */
            long totalAlgorithmsDuration = 0;

            boost::filesystem::path outputFolder{""};

            int frameNumber = 0;
            bool writeFrames = false;

            bool dontRenderFinalFrame = false;

            /**
             * The subclass specific main loop. All calculation is done here. <br>
             * Has to finally set the finalFrame which will then be rendered.
             */
            virtual void specificMainLoop() = 0;

            /**
             * Add some optional specific messages to the final frame after the main processing.
             */
            virtual void specificAddMessages();

            /**
             * Adds some text to the final frame.
             *
             * @param text The text to add.
             * @param x The left starting expectedPixel location.
             * @param y The upper starting expectedPixel location.
             */
            void addTextToFinalFrame(const std::string &text, int x, int y);

            /**
             * Adds some text to the final frame.
             *
             * @param text The text to add.
             * @param milliseconds The runtime in milliseconds.
             * @param x The left starting expectedPixel location.
             * @param y The upper starting expectedPixel location.
             */
            void addRuntimeToFinalFrame(const std::string &text, long milliseconds, int x, int y);

        public:

            virtual /**
			 * Initializer for the fields that are derived from the arguments.
			 */
            void init();

            /**
             * @set
             */
            void setCalculationScaleFactor(double calculationScaleFactor);

            /**
             * @set
             */
            void setRenderingScaleFactor(double renderingScaleFactor);

            /**
             * @constructor Directly sets the fields.
             *
             * @param inputFrame The video file name.
             * @param _windowName The name of the rendering window.
             * @param _calculationScaleFactor The scale factor of frame during calculation.
             * @param _renderingScaleFactor The scale factor of final frame during rendering.
             */
            explicit ImageSetup(
                std::string inputFrame = "../misc/test_frame.png",
                std::string outputFolder = "./results",
                std::string _windowName = "Camera Stabilization",
                double _calculationScaleFactor = 1,
                double _renderingScaleFactor = 0.5);

            virtual boost::program_options::variables_map fromCLI(int argc, const char **argv);;

            virtual void addInputOption(po::options_description *desc);

            /**
             * @destructor
             */
            virtual ~ImageSetup()  = default;

            /**
             * The main loop of the setup. <br>
             * Retrieves the next frame, does the processing, renders the results and measures the total duration.
             */
            void mainLoop();

            void setWindowMode(int flags);

            void setOutputFolder(const std::string &outputFolder);

            virtual void getNextFrame();

            virtual void addAdditionalOptions(po::options_description *desc);

            void setWriteFrames(bool writeFrames);
        };

        /**
         * Base class for all evaluation setups that run on a single image.
         * Wraps the main loop and field initializations.
         */
        class VideoSetup : public ImageSetup {

            /**
             * The video capture device.
             */
            cv::VideoCapture capture;
        public:

            /**
             * @constructor Directly sets the fields.
             *
             * @param _videoFileName The video file name.
             * @param _windowName The name of the rendering window.
             * @param _calculationScaleFactor The scale factor of frame during calculation.
             * @param _renderingScaleFactor The scale factor of final frame during rendering.
             */
            explicit VideoSetup(std::string _videoFileName = getDefaultVideoFile(),
                                std::string outputFolder = "./results",
                                std::string _windowName = "Camera Stabilization",
                                double _calculationScaleFactor = 1,
                                double _renderingScaleFactor = 0.5);

            boost::program_options::variables_map fromCLI(int argc, const char **argv) override;;

            void addInputOption(po::options_description *desc) override;

            /**
             * @destructor
             */
            ~VideoSetup() override;

            /**
             * @set
             */
            void setCapture(const std::string &file);

            void getNextFrame() override;

            void init() override;
        };

        cv::cuda::GpuMat pad(const cv::cuda::GpuMat &frame, int padding);
    }
}
#endif //WITH_OPENCV

#endif //CAMERASTABILIZATION_COMMONS_HPP
