//
// Created by brucknem on 13.01.21.
//

#include "CMakeConfig.h"

#ifdef WITH_OPENCV
#include "opencv2/opencv.hpp"
#include "opencv2/cudaimgproc.hpp"

#include "Commons.hpp"

#include <utility>
#include "boost/program_options.hpp"
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

#pragma region Helpers

std::string static_calibration::evaluation::getDefaultVideoFile() {
    std::string basePath = "/mnt/local_data/providentia/test_recordings/videos/";
    std::string filename = "s40_n_far_image_raw";
    std::string suffix = ".mp4";
    std::stringstream fullPath;
    fullPath << basePath << filename << suffix;
    return fullPath.str();
}

cv::VideoCapture static_calibration::evaluation::openVideoCapture(const std::string &videoFileName) {
    cv::VideoCapture cap(videoFileName);

    if (!cap.isOpened()) // if not success, exit program
    {
        throw std::invalid_argument("Cannot open " + videoFileName);
    }

    return cap;
}

std::string static_calibration::evaluation::durationInfo(const std::string &message, long milliseconds) {
    std::stringstream ss;
    ss << message << " - Duration: " << milliseconds << "ms - FPS: " << 1000. / milliseconds;
    return ss.str();
}

cv::Mat
static_calibration::evaluation::addText(const cv::Mat &frame, const std::string &text, double fontSize, int x, int y,
                                        cv::Scalar color) {
    cv::putText(frame, text, cv::Point(x, y + 20 * fontSize),
                cv::FONT_HERSHEY_COMPLEX_SMALL, fontSize, color, fontSize, cv::FONT_HERSHEY_SIMPLEX);
    return frame;
}

cv::cuda::GpuMat static_calibration::evaluation::pad(const cv::cuda::GpuMat &frame, int padding) {
    cv::cuda::GpuMat result;
    result.upload(pad(cv::Mat(frame), padding));
    return result;
}

cv::Mat static_calibration::evaluation::pad(const cv::Mat &frame, int padding) {
    return cv::Mat(frame,
                   cv::Rect(padding, padding, frame.cols - 2 * padding, frame.rows - 2 * padding));
}

cv::Mat static_calibration::evaluation::addRuntimeToFrame(const cv::Mat &_frame, const std::string &message,
                                                          long milliseconds, double fontSize, int x, int y) {
    return addText(_frame, durationInfo(message, milliseconds), fontSize, x, y);
}

double static_calibration::evaluation::getRandom01() {
    return static_cast<double>(rand() / static_cast<double>(RAND_MAX));
}

template<typename T>
std::vector<cv::Mat>
static_calibration::evaluation::concatenate(const std::initializer_list<T> &frames, int padding, cv::Size
size) {
    std::vector<cv::Mat> concatenated;
    cv::Mat resizeBuffer;

    cv::Size _size;
    for (const auto &frame : frames) {
        concatenated.emplace_back(cv::Mat(frame));
        if (_size.empty()) {
            if (size.empty()) {
                _size = concatenated[concatenated.size() - 1].size();
            } else {
                _size = size;
            }
        }
        if (!size.empty()) {
            cv::resize(concatenated[concatenated.size() - 1], concatenated[concatenated.size() - 1], _size);
        }

        concatenated[concatenated.size() - 1] = pad(concatenated[concatenated.size() - 1], padding);
    }

    return concatenated;
}

template<typename T>
cv::Mat static_calibration::evaluation::vconcat(const std::initializer_list<T> &frames, int padding, cv::Size size) {
    cv::Mat result;
    cv::vconcat(concatenate<T>(frames, padding, size), result);
    return result;
}

template cv::Mat
static_calibration::evaluation::vconcat(const std::initializer_list<cv::Mat> &, int padding, cv::Size size);

template cv::Mat
static_calibration::evaluation::vconcat(const std::initializer_list<cv::cuda::GpuMat> &, int padding, cv::Size size);

template<typename T>
cv::Mat static_calibration::evaluation::hconcat(const std::initializer_list<T> &frames, int padding, cv::Size size) {
    cv::Mat result;
    cv::hconcat(concatenate<T>(frames, padding, size), result);
    return result;
}

template cv::Mat
static_calibration::evaluation::hconcat(const std::initializer_list<cv::Mat> &, int padding, cv::Size size);

template cv::Mat
static_calibration::evaluation::hconcat(const std::initializer_list<cv::cuda::GpuMat> &, int padding, cv::Size size);

cv::Mat static_calibration::evaluation::MatOfSize(cv::Size size, int type) {
    return cv::Mat(cv::Mat::zeros(std::move(size), type));
}

cv::cuda::GpuMat static_calibration::evaluation::GpuMatOfSize(cv::Size size, int type) {
    cv::cuda::GpuMat mat;
    mat.upload(cv::Mat(cv::Mat::zeros(std::move(size), type)));
    return mat;
}

cv::cuda::GpuMat static_calibration::evaluation::cvtColor(cv::cuda::GpuMat frame, int colorSpace) {
    cv::cuda::GpuMat result;
    cv::cuda::cvtColor(frame, result, colorSpace);
    return result;
}

std::string static_calibration::evaluation::getNowSuffix() {
    auto now = getNow();
    auto date = now.date();
    auto day = now.time_of_day();
    std::stringstream ss;
    ss << "_";
    ss << boost::gregorian::to_iso_extended_string(date);
    ss << "_";
    ss << std::setw(2) << std::setfill('0') << day.hours() << "-";
    ss << std::setw(2) << std::setfill('0') << day.minutes() << "-";
    ss << std::setw(2) << std::setfill('0') << day.seconds();
    return ss.str();
}

boost::posix_time::ptime static_calibration::evaluation::getNow() {
    return boost::posix_time::second_clock::local_time();
}

#pragma endregion Helpers

#pragma region RunnablesCommons

static_calibration::evaluation::ImageSetup::ImageSetup(std::string inputFrame,
                                                       std::string outputFolder,
                                                       std::string windowName,
                                                       double _calculationScaleFactor,
                                                       double _renderingScaleFactor)
        : inputResource(std::move(inputFrame)), calculationScaleFactor(_calculationScaleFactor),
          renderingScaleFactor(_renderingScaleFactor),
          windowName(std::move(windowName)),
          outputFolder(outputFolder) {}

void static_calibration::evaluation::ImageSetup::init() {
    renderingScaleFactor /= calculationScaleFactor;
    if (!dontRenderFinalFrame) {
        cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);
        cv::moveWindow(windowName, 50, 10);
        srand(static_cast <unsigned> (time(0)));
    }
    frameCPU = cv::imread(inputResource);
}

void static_calibration::evaluation::ImageSetup::setWindowMode(int flags) {
    cv::destroyWindow(windowName);
    cv::namedWindow(windowName, flags);
}

void
static_calibration::evaluation::ImageSetup::addRuntimeToFinalFrame(const std::string &text, long milliseconds, int x,
                                                                   int y) {
    addTextToFinalFrame(durationInfo(text, milliseconds), x, y);
}

void static_calibration::evaluation::ImageSetup::addTextToFinalFrame(const std::string &text, int x, int y) {
    addText(finalFrame, text, 1, x, y);
}

void static_calibration::evaluation::ImageSetup::getNextFrame() {
//pass
}

void static_calibration::evaluation::ImageSetup::mainLoop() {
    init();
    while (true) {
        getNextFrame();
        if (frameCPU.empty()) {
            std::cout << "No new frame available. Stopped rendering loop." << std::endl;
            break;
        }
        finalFrame = cv::Mat();
        cv::resize(frameCPU, frameCPU, cv::Size(), calculationScaleFactor, calculationScaleFactor);
        frameGPU.upload(frameCPU);

        totalAlgorithmsDuration = 0;
        specificMainLoop();

        if (totalAlgorithmsDuration > 0) {
            addRuntimeToFinalFrame("Algorithms total", totalAlgorithmsDuration, 5, finalFrame.rows - 40);
        }

        specificAddMessages();

//		addRuntimeToFinalFrame("Frame " + std::to_string(frameNumber), getTotalMilliseconds(), 5, finalFrame.rows - 20);

        if (writeFrames) {
            finalFrame.convertTo(finalFrame, CV_8UC4);
            std::stringstream frameId;
            frameId << std::setw(4) << std::setfill('0');
            frameId << frameNumber;
            if (!boost::filesystem::is_directory(outputFolder / "frames")) {
                boost::filesystem::create_directories(outputFolder / "frames");
            }
            boost::filesystem::path outFile = outputFolder / "frames" / ("frame_" + frameId.str() + ".jpg");
            cv::imwrite(outFile.string(), finalFrame, {cv::IMWRITE_PNG_COMPRESSION, 9});
        }

        if (!dontRenderFinalFrame) {
            cv::resize(finalFrame, finalFrame, cv::Size(), renderingScaleFactor, renderingScaleFactor);
            cv::imshow(windowName, finalFrame);
        }
        pressedKey = cv::waitKey(1);
        if (pressedKey == (int) ('q')) {
            break;
        }

        frameNumber++;
    }
}

void static_calibration::evaluation::ImageSetup::specificAddMessages() {
    // Empty stub for optional override.
}

void static_calibration::evaluation::ImageSetup::setCalculationScaleFactor(double _calculationScaleFactor) {
    calculationScaleFactor = _calculationScaleFactor;
}

void static_calibration::evaluation::ImageSetup::setRenderingScaleFactor(double _renderingScaleFactor) {
    renderingScaleFactor = _renderingScaleFactor;
}

void static_calibration::evaluation::ImageSetup::setOutputFolder(const std::string &_outputFolder) {
    outputFolder = _outputFolder;
    if (!boost::filesystem::is_directory(outputFolder)) {
        boost::filesystem::create_directories(outputFolder);
    }
}

boost::program_options::variables_map static_calibration::evaluation::ImageSetup::fromCLI(int argc, const char **argv) {
    po::options_description desc("Allowed options");
    desc.add_options()
            ("help,h", "produce help message");
    addInputOption(&desc);
    desc.add_options()
            ("output,o", po::value<std::string>()->default_value("./results"), "The output folder.")
            ("csf,c", po::value<double>()->default_value(1), "The calculation scale factor.")
            ("rsf,r", po::value<double>()->default_value(0.5), "The rendering scale factor.")
            ("render,d", po::bool_switch(&dontRenderFinalFrame),
             "Flag: Render the final frame or a black placeholder window.")
            ("writeFrames,f", po::bool_switch(&writeFrames), "Flag: Write the writeFrames to the output folder.")
            ("window-name,w", po::value<std::string>()->default_value("Camera Stabilization"),
             "The name of the OpenCV window.");
    addAdditionalOptions(&desc);

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help") > 0) {
        std::cout << desc << std::endl;
        exit(EXIT_SUCCESS);
    }

    inputResource = vm["input"].as<std::string>();
    setOutputFolder(vm["output"].as<std::string>());
    windowName = vm["window-name"].as<std::string>();
    renderingScaleFactor = vm["rsf"].as<double>();
    calculationScaleFactor = vm["csf"].as<double>();
    frameCPU = cv::imread(inputResource);
    return vm;
}

void static_calibration::evaluation::ImageSetup::addInputOption(po::options_description *desc) {
    desc->add_options()("input,i", po::value<std::string>()->default_value("../misc/test_frame.png"),
                        "The input resource.");
}

void static_calibration::evaluation::ImageSetup::addAdditionalOptions(po::options_description *desc) {
    // Empty for override
}

void static_calibration::evaluation::ImageSetup::setWriteFrames(bool writeFrames) {
    ImageSetup::writeFrames = writeFrames;
}

#pragma endregion RunnablesCommons

void static_calibration::evaluation::VideoSetup::setCapture(const std::string &file) {
    if (capture.isOpened()) {
        capture.release();
    }
    capture = openVideoCapture(file);
}

static_calibration::evaluation::VideoSetup::VideoSetup(std::string _videoFileName,
                                                       std::string outputFolder,
                                                       std::string _windowName,
                                                       double _calculationScaleFactor,
                                                       double _renderingScaleFactor)
        : ImageSetup(std::move(_videoFileName), std::move(outputFolder), std::move(_windowName),
                     _calculationScaleFactor,
                     _renderingScaleFactor) {}

static_calibration::evaluation::VideoSetup::~VideoSetup() {
    capture.release();
    cv::destroyAllWindows();
}

void static_calibration::evaluation::VideoSetup::getNextFrame() {
    capture >> frameCPU;
}

boost::program_options::variables_map static_calibration::evaluation::VideoSetup::fromCLI(int argc, const char **argv) {
    return ImageSetup::fromCLI(argc, argv);
}

void static_calibration::evaluation::VideoSetup::addInputOption(po::options_description *desc) {
    desc->add_options()("input,i", po::value<std::string>()->default_value(getDefaultVideoFile()),
                        "The input resource.");
}

void static_calibration::evaluation::VideoSetup::init() {
    ImageSetup::init();
    capture = openVideoCapture(inputResource);
}

#endif //WITH_OPENCV
