//
// Created by brucknem on 02.02.21.
//

#include <thread>
#include <boost/algorithm/string/split.hpp>
#include "Commons.hpp"
#include "Eigen/Dense"
#include "StaticCalibration/camera/RenderingPipeline.hpp"
#include "StaticCalibration/objects/WorldObject.hpp"
#include "StaticCalibration/objects/ObjectsLoading.hpp"
#include "StaticCalibration/CameraPoseEstimation.hpp"
#include "CSVWriter.hpp"

#include <boost/foreach.hpp>
#include <boost/algorithm/string/trim.hpp>
#include "glog/logging.h"

using namespace static_calibration::evaluation;

/**
 * Setup to visualize the rendering pipeline.
 */
class Setup : public static_calibration::evaluation::ImageSetup {
public:
    /**
     * The itnrinsics of the pinhole camera model.
     */
    std::vector<double> initialIntrinsics;
    std::vector<double> intrinsics;
    std::shared_ptr<static_calibration::calibration::CameraPoseEstimation> estimator;

    /**
     * The [width, height] of the image.
     */
    Eigen::Vector2i imageSize{1920, 1200};

    /**
     * Some [x, y, z] translation of the camera in world space.
     */
    Eigen::Vector3d initialTranslation{
            695962.92502753110602498055, 5346500.60284730326384305954 - 1000, 538.63933146729164036515
    };
    Eigen::Vector3d translation = {initialTranslation};

    /**
     * Some [x, y, z] euler angle rotation of the camera around the world axis
     */
    Eigen::Vector3d initialRotation{85, 0, -160};
    Eigen::Vector3d rotation = {initialRotation};

    /**
     * Flag for the background.
     */
    int trackbarBackground = 4;

    int trackbarShowIds = 0;

    int evaluationRun = -1;
    int weightScale = 100;
    int maxWeightScale = 100;

    int lambdaIndex = -1;
    std::vector<double> lambdaOptions{4.};

    int rotationIndex = 0;
    std::vector<double> rotationOptions{50.};

    int runsPerScale = 250;

    /**
     * The objects from the HD map.
     */
    std::vector<static_calibration::calibration::WorldObject> objects;

    bool optimizationFinished = false;

    bool renderObjects = true;

    std::string pixelsFile, objectsFile;
    boost::filesystem::path evaluationPath;
    CSVWriter *extrinsicParametersWriter;
    CSVWriter *optimizationRunWriter;

    explicit Setup() : ImageSetup() {
        google::InitGoogleLogging("Camera Pose Estimation");
    }

    boost::program_options::variables_map fromCLI(int argc, const char **argv) override {
        auto vm = ImageSetup::fromCLI(argc, argv);
        if (vm.count("intrinsics") <= 0) {
            std::cout << "Provide intrinsics.";
            exit(EXIT_FAILURE);
        }
        std::vector<std::string> rawIntrinsics;
        boost::split(rawIntrinsics, vm["intrinsics"].as<std::string>(), [](char c) { return c == ','; });
        if (rawIntrinsics.size() != 5) {
            std::cout << "Provide intrinsics as comma separated list in format \"f_x,0,c_x,0,f_y,c_y,0,skew,1\".";
            exit(EXIT_FAILURE);
        }
        BOOST_FOREACH(std::string value, rawIntrinsics) {
                        intrinsics.emplace_back(boost::lexical_cast<double>(boost::trim_copy(value)));
                    };
        initialIntrinsics = intrinsics;
        std::cout << static_calibration::camera::getIntrinsicsMatrix(initialIntrinsics.data()) << std::endl;
        pixelsFile = vm["pixels"].as<std::string>();
//		initialRotation.z() = vm["z_init"].as<int>();
        return vm;
    }

    void addAdditionalOptions(po::options_description *desc) override {
        desc->add_options()
                ("intrinsics,t", po::value<std::string>(),
                 "The intrinsic parameters of the camera as comma separated list in format \"f_x,0,c_x,0,f_y,c_y,0,skew,"
                 "1\".")
                ("pixels,p", po::value<std::string>(), "The pixels mapping file.");
//			("z_init,z", po::value<int>()->default_value(0),
//			 "The initial z rotation.");
    }

    void initCSVWriters() {
        auto pixelsFilePath = boost::filesystem::path(pixelsFile);
        evaluationPath =
                outputFolder / "StaticCalibration" / pixelsFilePath.parent_path().filename();
        if (!boost::filesystem::is_directory(evaluationPath)) {
            boost::filesystem::create_directories(evaluationPath);
        }
        auto suffix = getNowSuffix();
        extrinsicParametersWriter = new CSVWriter(
                evaluationPath / (pixelsFilePath.filename().string() + suffix + ".csv"));

        *extrinsicParametersWriter << "Run"
                                   << "Correspondences"
                                   << "Penalize Scale [Lambdas]"
                                   << "Penalize Scale [Rotation]"
                                   << "Penalize Scale [Weights]"
                                   << "Valid Solution"
                                   << "Loss"
                                   << "Loss [Correspondences]"
                                   << "Loss [Lambdas]"
                                   << "Loss [Intrinsics]"
                                   << "Loss [Rotations]"
                                   << "Loss [Weights]"
                                   << "Translation [x]"
                                   << "Translation [y]"
                                   << "Translation [z]"
                                   << "Rotation [x]"
                                   << "Rotation [y]"
                                   << "Rotation [z]"
                                   << "Focal Length"
                                   << "Focal Length [Ratio]"
                                   << "Principal Point [u]"
                                   << "Principal Point [v]"
                                   << "Skew"
                                   << "Weights [Avg]"
                                   << "Weights [Min]"
                                   << "Weights [Max]"
                                   << newline
                                   << flush;
    }

    void init() override {
        ImageSetup::init();

        objectsFile = (boost::filesystem::path(inputResource).parent_path() / "objects.yaml").string();

        objects = static_calibration::calibration::loadObjects(objectsFile, pixelsFile, imageSize);
        estimator = std::make_shared<static_calibration::calibration::CameraPoseEstimation>();
        estimator->addWorldObjects(objects);
        if (!dontRenderFinalFrame) {
            cv::createTrackbar("Background", windowName, &trackbarBackground, 10);
            cv::createTrackbar("Show Ids", windowName, &trackbarShowIds, 1);
        }
        initCSVWriters();
//		setWriteFrames(true);
    }

    void render(std::string id, Eigen::Vector3d vector, const cv::Vec3d &color, bool showId) {
        render(id, vector.x(), vector.y(), vector.z(), color, showId);
    }

    void render(std::string id, double x, double y, double z, const cv::Vec3d &color, bool showId) {
        Eigen::Vector4d vector{x, y, z, 1};
        Eigen::Vector4d vectorInCameraSpace = static_calibration::camera::toCameraSpace(
                translation.data(), rotation.data(), vector.data());

        if (std::abs(vectorInCameraSpace.z()) > 2000) {
            return;
        }

        bool flipped;
        auto pixel = static_calibration::camera::render(translation, rotation,
                                                        intrinsics, vector, color,
                                                        finalFrame, flipped);

        if (flipped) {
            return;
        }

//		if (pixel.x() >= 0 && pixel.x() < imageSize[0] &&
//			pixel.y() >= 0 && pixel.y() < imageSize[1]) {
//			std::cout << id << std::endl;
//		}

        std::stringstream ss;
        ss << std::fixed;
        ss << id;
//		ss << ": " << x << "," << y << "," << z;

        if (showId && trackbarShowIds == 1) {
            addTextToFinalFrame(ss.str(), pixel.x(), imageSize[1] - 1 - pixel.y());
        }
    }

protected:
    void initFinalFrame() {
        finalFrame = frameCPU.clone();
        std::vector<cv::Mat> matChannels;
        cv::split(finalFrame, matChannels);
        // create alpha channel
        cv::Mat alpha = cv::Mat::ones(frameCPU.size(), CV_8UC1);
        matChannels.push_back(alpha);
        cv::merge(matChannels, finalFrame);
        finalFrame.convertTo(finalFrame, CV_64FC4, 1. / 255.);
        finalFrame = finalFrame * (trackbarBackground / 10.);
    }

    void renderText() {
        cv::rectangle(finalFrame, {0, finalFrame.rows - 125}, {600, finalFrame.rows}, {0, 0, 0}, -1);
        cv::rectangle(finalFrame, {1345, finalFrame.rows - 190 - 24}, {finalFrame.cols, finalFrame.rows}, {0, 0, 0},
                      -1);

        addTextToFinalFrame("RED dots: Unmapped objects", 5, finalFrame.rows - 120);
        addTextToFinalFrame("GREEN dots: Mapped objects", 5, finalFrame.rows - 100);
        addTextToFinalFrame("Weight Scale: " + std::to_string(powBase2(weightScale)), 5, finalFrame.rows - 80);
        addTextToFinalFrame(
                "Run: " + std::to_string(evaluationRun + (lambdaIndex * runsPerScale) + ((lambdaOptions.size() *
                                                                                          runsPerScale) *
                                                                                         rotationIndex)) +
                "/"
                + std::to_string(lambdaOptions.size() * rotationOptions.size() * runsPerScale), 5,
                finalFrame.rows - 60);
        addTextToFinalFrame("Lambda: "
                            + std::to_string(lambdaOptions[lambdaIndex]), 5, finalFrame.rows - 40);
        addTextToFinalFrame("Rotation: "
                            + std::to_string(rotationOptions[rotationIndex]), 5, finalFrame.rows - 20);

        std::stringstream ss;
        ss << *estimator;
        std::string line;
        int i = 0;
        while (getline(ss, line)) {
            addTextToFinalFrame(line, 1350, finalFrame.rows - 190 + i++ * 24);
        }
    }

    void render() {
        for (const auto &worldObject: objects) {
            bool idShown = false;
            auto centerLine = worldObject.getCenterLine();
            if (centerLine.empty()) {
                for (const auto &point : worldObject.getPoints()) {
                    Eigen::Vector3d p = point.getPosition();
                    cv::Vec3d color = {0, 0, 1};
                    render(worldObject.getId(), p, color, !idShown);
                    idShown = true;
                }
            }
            for (const auto &point : centerLine) {
                Eigen::Vector3d p = point.getPosition();
                cv::Vec3d color = {0, 0, 1};
                if (point.hasExpectedPixel()) {
                    color = {0, 1, 0};
                }
                render(worldObject.getId(), p, color, !idShown);
                idShown = true;
            }
        }
    }

    double powBase2(double val) {
        return std::pow(2, val);
    }

    void writeToCSV() {
        if (evaluationRun > -1) {
            auto weights = estimator->getWeights();

            double min_w = 1e100;
            double max_w = -1e100;
            double sum_w = 0;
            for (const auto &weight : weights) {
                if (weight < min_w) {
                    min_w = weight;
                }
                if (weight > max_w) {
                    max_w = weight;
                }
                sum_w += weight;
            }

            *extrinsicParametersWriter << evaluationRun
                                       << (int) weights.size()
                                       << lambdaOptions[lambdaIndex]
                                       << rotationOptions[rotationIndex]
                                       << (powBase2(weightScale))
                                       << estimator->hasFoundValidSolution()
                                       << estimator->getTotalLoss()
                                       << estimator->getCorrespondencesLoss()
                                       << estimator->getLambdasLoss()
                                       << estimator->getIntrinsicsLoss()
                                       << estimator->getRotationsLoss()
                                       << estimator->getWeightsLoss()
                                       << translation
                                       << rotation
                                       << estimator->getIntrinsics()
                                       << sum_w / weights.size()
                                       << min_w
                                       << max_w
                                       << newline;
        }
    }

    void doEvaluationIncrementations() {
        if (evaluationRun % runsPerScale == 0) {
            evaluationRun = 0;
            lambdaIndex++;
            if (lambdaIndex >= lambdaOptions.size()) {
                lambdaIndex = 0;
                rotationIndex++;
                if (rotationIndex >= rotationOptions.size()) {
                    exit(EXIT_SUCCESS);
                }
            }
        }
    }

    void specificMainLoop() override {
        initFinalFrame();

        optimizationFinished = estimator->isEstimationFinished();
        translation = estimator->getTranslation();
        rotation = estimator->getRotation();
        intrinsics = estimator->getIntrinsics();

        if (optimizationFinished) {
            writeToCSV();
            evaluationRun++;
            doEvaluationIncrementations();

//			estimator.reset();
//			auto count = estimator.unique();
//			estimator = std::make_shared<static_calibration::StaticCalibration::CameraPoseEstimation>(intrinsics, true,
//																						powBase2(weightScale));
            estimator->setWeightPenalizeScale(powBase2(weightScale));
//			estimator->addWorldObjects(objects);
//			estimator->setLambdaScale(lambdaOptions[lambdaIndex]);
//			estimator->setRotationScale(rotationOptions[rotationIndex]);

            estimator->clearWorldObjects();
            estimator->addWorldObjects(objects);
            estimator->guessIntrinsics(initialIntrinsics);
//			estimator->guessRotation(initialRotation);
//			estimator->guessTranslation(initialTranslation);

            optimizationFinished = false;
            estimator->estimateAsync(true);
        }

        render();
//		renderText();
    }
};

int main(int argc, char const *argv[]) {
    Setup setup;
    setup.fromCLI(argc, argv);
    setup.setRenderingScaleFactor(1);
    setup.setOutputFolder("./stabilization/");
    setup.mainLoop();
    return 0;
}
