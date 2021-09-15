//
// Created by brucknem on 02.02.21.
//

#include "StaticCalibration/CameraPoseEstimationBase.hpp"

#include "ceres/autodiff_cost_function.h"
#include <thread>
#include <StaticCalibration/residuals/CorrespondenceWithIntrinsicsResidual.hpp>
#include <utility>

namespace static_calibration {
    namespace calibration {

        CameraPoseEstimationBase::CameraPoseEstimationBase(const std::vector<double> &intrinsics) {
            setIntrinsics(intrinsics);
        }

        const Eigen::Vector3d &CameraPoseEstimationBase::getTranslation() const {
            return translation;
        }

        Eigen::Vector3d CameraPoseEstimationBase::clearRotation(const Eigen::Vector3d &rotation) {
            Eigen::Vector3d resultRotation = rotation;
            for (int i = 0; i < 3; i++) {
                while (resultRotation[i] < -180 || resultRotation[i] > 180) {
                    if (resultRotation[i] < -180) {
                        resultRotation[i] += 360;
                    }
                    if (resultRotation[i] > 180) {
                        resultRotation[i] -= 360;
                    }
                }
            }
            return resultRotation;
        }

        Eigen::Vector3d CameraPoseEstimationBase::getRotation() const {
            return clearRotation(rotation);
        }

        void CameraPoseEstimationBase::calculateInitialGuess() {
            if (!hasTranslationGuess) {
                Eigen::Vector3d mean = calculateMean();

                initialTranslation = mean;
                initialTranslation.z() += initialDistanceFromMean;
                translation = initialTranslation;
            }

            if (!hasRotationGuess) {
                double x = 35.;
                initialRotation = {generateRandomNumber(-x, x),
                                   generateRandomNumber(-x, x),
                                   generateRandomNumber(-x, x)};
                rotation = initialRotation;
            }
        }

        Eigen::Vector3d CameraPoseEstimationBase::calculateMean() {
            Eigen::Vector3d meanVector(0, 0, 0);
            auto parametricPoints = dataSet.getParametricPoints<Object>();
            for (const auto &worldObject: parametricPoints) {
                meanVector += worldObject.getOrigin();
            }
            return meanVector / parametricPoints.size();
        }

        std::thread CameraPoseEstimationBase::estimateAsync(bool logSummary) {
            auto thread = std::thread(&CameraPoseEstimationBase::estimate, this, logSummary);
            thread.detach();
            return thread;
        }

        ceres::Problem CameraPoseEstimationBase::solveProblem(bool logSummary) {
            auto problem = createProblem();
            auto options = setupOptions(logSummary);
            Solve(options, &problem, &summary);
            evaluateAllResiduals(problem);
            evaluateCorrespondenceResiduals(problem);
            evaluateExplicitRoadMarkResiduals(problem);
            evaluateLambdaResiduals(problem);
            evaluateRotationResiduals(problem);
            evaluateWeightResiduals(problem);
            return problem;
        }

        std::vector<double> CameraPoseEstimationBase::getLambdas() {
            std::vector<double> lambdas;
            throw std::logic_error("Not implemented");

//            for (const auto &worldObject : worldObjects) {
//                std::cout << "World Object: " << worldObject.getId() << std::endl;
//             TODO fix add lambdas
//                for (const auto &point : worldObject.getCenterLine()) {
//                    std::cout << *point.getLambda() << " / " << worldObject.getLength() << std::endl;
//                    lambdas.emplace_back(*point.getLambda());
//                }
//            }
            return lambdas;
        }

        void CameraPoseEstimationBase::estimate(bool logSummary) {
            optimizationFinished = false;
            foundValidSolution = false;
            int i = 0;
            for (; i < maxTriesUntilAbort; i++) {
                resetParameters();
                calculateInitialGuess();
                solveProblem(logSummary);

                bool invalidSolution = rotationsLoss > 1e-6 || intrinsicsLoss > 5;
                bool invalidCorrespondencesLoss = correspondencesLoss > getCorrespondenceLossUpperBound();
//                invalidSolution = invalidSolution || invalidCorrespondencesLoss;
//                bool invalidLambdas = lambdasLoss > 10;
                double rotationDiff = (initialRotation - rotation).norm();
//                std::cout << rotationDiff << std::endl;
                bool invalidRotation = rotationDiff > 50;
                double translationDiff = (initialTranslation - translation).norm();
//                std::cout << translationDiff << std::endl;
                bool invalidTranslation = translationDiff > 100;

//                invalidSolution = invalidSolution || invalidLambdas;
//                invalidSolution = invalidSolution || invalidCorrespondencesLoss;
                invalidSolution = invalidSolution || invalidTranslation;
                invalidSolution = invalidSolution || invalidRotation;

                if (invalidSolution) {
                    continue;
                }
                double originalPenalize = lambdaResidualScalingFactor;
                lambdaResidualScalingFactor = originalPenalize * 10000;
                solveProblem(logSummary);
                lambdaResidualScalingFactor = originalPenalize;
                break;
            }

            foundValidSolution = i < maxTriesUntilAbort;
            optimizationFinished = true;
            if (logSummary) {
                std::cout << *this << std::endl;
            }

        }

        ceres::Solver::Options CameraPoseEstimationBase::setupOptions(bool logSummary) {
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
//			options.trust_region_strategy_type = ceres::DOGLEG;
//			options.use_nonmonotonic_steps = true;
//			options.max_num_consecutive_invalid_steps = 15;
//			options.max_num_iterations = weights.size();
            options.max_num_iterations = 400;
            auto processorCount = std::thread::hardware_concurrency();
            if (processorCount == 0) {
                processorCount = 8;
            }
            options.num_threads = (int) processorCount;
//            options.num_threads = 1;
            options.minimizer_progress_to_stdout = logSummary;
            options.update_state_every_iteration = true;
            if (!logSummary) {
                options.logging_type = ceres::SILENT;
            }
            return options;
        }

        ceres::ScaledLoss *CameraPoseEstimationBase::getScaledHuberLoss(double scale) {
            return getScaledHuberLoss(1.0, scale);
        }

        ceres::ScaledLoss *CameraPoseEstimationBase::getScaledHuberLoss(double huber, double scale) {
            return new ceres::ScaledLoss(
                    new ceres::HuberLoss(huber),
                    scale,
                    ceres::TAKE_OWNERSHIP
            );
        }

        void CameraPoseEstimationBase::resetParameters() {
            for (const auto &point: dataSet.getParametricPoints<Object>()) {
                *point.getLambda() = 0;
            }
            for (const auto &point: dataSet.getParametricPoints<RoadMark>()) {
                *point.getLambda() = 0;
            }
        }

        // TODO do not recreate; Create only once by dedicated call and reset values in residual blocks.
        // TODO This should remove the memory leak!!!
        ceres::Problem CameraPoseEstimationBase::createProblem() {
            auto problem = ceres::Problem();
            for (auto p: weights) {
                delete p;
            }
            weights.clear();
            correspondenceResiduals.clear();
            explicitRoadMarkResiduals.clear();
            weightResiduals.clear();
            lambdaResiduals.clear();

            for (const auto &point: dataSet.getParametricPoints<Object>()) {
                weights.emplace_back(new double(1));
                correspondenceResiduals.emplace_back(
                        addCorrespondenceResidualBlock(problem, point, new ceres::HuberLoss(100.0)));
                lambdaResiduals.emplace_back(addLambdaResidualBlock(problem, point));
                weightResiduals.emplace_back(addWeightResidualBlock(problem, weights[weights.size() - 1]));
            }

            for (const auto &point: dataSet.getParametricPoints<RoadMark>()) {
                weights.emplace_back(new double(1));
                explicitRoadMarkResiduals.emplace_back(
                        addCorrespondenceResidualBlock(problem, point, new ceres::HuberLoss(1.0)));
                lambdaResiduals.emplace_back(addLambdaResidualBlock(problem, point));
                weightResiduals.emplace_back(addWeightResidualBlock(problem, weights[weights.size() - 1]));
            }

            addRotationConstraints(problem);

//			std::cout << "Residuals: " << problem.NumResidualBlocks() << std::endl;
            return problem;
        }

        ceres::ResidualBlockId
        CameraPoseEstimationBase::addWeightResidualBlock(ceres::Problem &problem, double *weight) const {
            return problem.AddResidualBlock(
                    residuals::DistanceResidual::create(1),
                    getScaledHuberLoss(weightResidualScalingFactor),
                    weight
            );
        }

        ceres::ResidualBlockId
        CameraPoseEstimationBase::addLambdaResidualBlock(ceres::Problem &problem, const ParametricPoint &point) const {
            return problem.AddResidualBlock(
                    residuals::DistanceFromIntervalResidual::create(point.getLambdaMin(), point.getLambdaMax()),
                    getScaledHuberLoss(lambdaResidualScalingFactor),
                    point.getLambda()
            );
        }

        void CameraPoseEstimationBase::addRotationConstraints(ceres::Problem &problem) {
            rotationResiduals.clear();
            rotationResiduals.emplace_back(problem.AddResidualBlock(
                    static_calibration::calibration::residuals::DistanceFromIntervalResidual::create(60, 110),
                    getScaledHuberLoss(rotationResidualScalingFactor),
                    &rotation.x()
            ));
            rotationResiduals.emplace_back(problem.AddResidualBlock(
                    static_calibration::calibration::residuals::DistanceFromIntervalResidual::create(-10, 10),
                    getScaledHuberLoss(rotationResidualScalingFactor),
                    &rotation.y()
            ));
        }

        template<>
        void CameraPoseEstimationBase::guessRotation(const Eigen::Vector3d &value) {
            hasRotationGuess = true;
            initialRotation = value;
            rotation = value;
        }

        template<>
        void CameraPoseEstimationBase::guessTranslation(const Eigen::Vector3d &value) {
            hasTranslationGuess = true;
            initialTranslation = value;
            translation = value;
            initialDistanceFromMean = 0;
        }

        template<>
        void CameraPoseEstimationBase::guessRotation(const std::vector<double> &value) {
            guessRotation<Eigen::Vector3d>(Eigen::Vector3d(value.data()));
        }

        template<>
        void CameraPoseEstimationBase::guessTranslation(const std::vector<double> &value) {
            guessTranslation<Eigen::Vector3d>(Eigen::Vector3d(value.data()));
        }

        bool CameraPoseEstimationBase::isEstimationFinished() const {
            return optimizationFinished;
        }

        std::ostream &operator<<(std::ostream &os, const CameraPoseEstimationBase *estimator) {
            os << *estimator;
            return os;
        }

        std::ostream &operator<<(std::ostream &os, const CameraPoseEstimationBase &estimator) {
            os << "Translation:" << std::endl;
            os << "From:       " << printVectorRow(estimator.initialTranslation) << std::endl;
            os << "To:         " << printVectorRow(estimator.translation) << std::endl;
            os << "Difference: " << printVectorRow(estimator.translation - estimator.initialTranslation)
               << std::endl;

            os << "Rotation:" << std::endl;
            os << "From:       " << printVectorRow(estimator.initialRotation) << std::endl;
            os << "To:         " << printVectorRow(estimator.getRotation()) << std::endl;
            os << "Difference: " << printVectorRow(estimator.getRotation() - estimator.initialRotation)
               << std::endl;

            os << "Intrinsics:" << std::endl;
            os << "From:       " << printVectorRow(estimator.initialIntrinsics) << std::endl;
            os << "To:         " << printVectorRow(estimator.intrinsics) << std::endl;
            std::vector<double> intrinicsDifference = estimator.initialIntrinsics;
            for (int i = 0; i < intrinicsDifference.size(); i++) {
                intrinicsDifference[i] -= estimator.intrinsics[i];
            }
            os << "Difference: " << printVectorRow(intrinicsDifference) << std::endl;

//            os << "Weights:" << std::endl;
//            for (const auto &weight : estimator.weights) {
//                os << *weight << ", ";
//            }
            return os;
        }

        const std::vector<double> &CameraPoseEstimationBase::getIntrinsics() const {
            return intrinsics;
        }

        void CameraPoseEstimationBase::setWeightPenalizeScale(double value) {
            weightResidualScalingFactor = value;
        }

        std::vector<double> CameraPoseEstimationBase::getWeights() {
            std::vector<double> result;
            result.reserve(weights.size());
            std::transform(std::begin(weights), std::end(weights),
                           std::back_inserter(result), [](const double *weight) { return *weight; }
            );
            return result;
        }

        double
        CameraPoseEstimationBase::evaluate(ceres::Problem &problem,
                                           const ceres::Problem::EvaluateOptions &evaluateOptions) {
            double loss;
            std::vector<double> residuals;
            problem.Evaluate(evaluateOptions, &loss, &residuals, nullptr, nullptr);
            return loss;
        }

        double CameraPoseEstimationBase::evaluate(ceres::Problem &problem, const std::vector<ceres::ResidualBlockId>
        &blockIds) {
            auto evaluateOptions = ceres::Problem::EvaluateOptions();
            evaluateOptions.residual_blocks = blockIds;
            return evaluate(problem, evaluateOptions);
        }

        void CameraPoseEstimationBase::evaluateAllResiduals(ceres::Problem &problem) {
            totalLoss = evaluate(problem);
        }

        void CameraPoseEstimationBase::evaluateCorrespondenceResiduals(ceres::Problem &problem) {
            correspondencesLoss = evaluate(problem, correspondenceResiduals);
        }

        void CameraPoseEstimationBase::evaluateExplicitRoadMarkResiduals(ceres::Problem &problem) {
            explicitRoadMarksLoss = evaluate(problem, explicitRoadMarkResiduals);
        }

        void CameraPoseEstimationBase::evaluateLambdaResiduals(ceres::Problem &problem) {
            lambdasLoss = evaluate(problem, lambdaResiduals);
        }

        void CameraPoseEstimationBase::evaluateWeightResiduals(ceres::Problem &problem) {
            weightsLoss = evaluate(problem, weightResiduals);
        }

        void CameraPoseEstimationBase::evaluateRotationResiduals(ceres::Problem &problem) {
            rotationsLoss = evaluate(problem, rotationResiduals);
        }

        bool CameraPoseEstimationBase::hasFoundValidSolution() const {
            return foundValidSolution;
        }

        double CameraPoseEstimationBase::getLambdasLoss() const {
            return lambdasLoss;
        }

        double CameraPoseEstimationBase::getCorrespondencesLoss() const {
            return correspondencesLoss;
        }

        double CameraPoseEstimationBase::getExplicitRoadMarksLoss() const {
            return explicitRoadMarksLoss;
        }

        double CameraPoseEstimationBase::getRotationsLoss() const {
            return rotationsLoss;
        }

        double CameraPoseEstimationBase::getWeightsLoss() const {
            return weightsLoss;
        }

        double CameraPoseEstimationBase::getTotalLoss() const {
            return totalLoss;
        }

        ceres::ResidualBlockId
        CameraPoseEstimationBase::addCorrespondenceResidualBlock(ceres::Problem &problem,
                                                                 const ParametricPoint &point,
                                                                 ceres::LossFunction *lossFunction) {
            // This is a mock function used only for override.
            return ceres::ResidualBlockId(-1);
        }

        void CameraPoseEstimationBase::setIntrinsics(const std::vector<double> &_intrinsics) {
            intrinsics = _intrinsics;
            if (intrinsics.size() == 4) {
                intrinsics.emplace_back(0);
            }
            if (intrinsics.size() != 5) {
                throw std::invalid_argument("The intrinsics need to be 5 values.");
            }
            initialIntrinsics = intrinsics;
        }

        double CameraPoseEstimationBase::getIntrinsicsLoss() const {
            return intrinsicsLoss;
        }

        int CameraPoseEstimationBase::getCorrespondenceLossUpperBound() const {
            return (int) weightResiduals.size();
        }

        const objects::DataSet &CameraPoseEstimationBase::getDataSet() const {
            return dataSet;
        }

        void CameraPoseEstimationBase::setDataSet(const objects::DataSet &dataSet) {
            CameraPoseEstimationBase::dataSet = dataSet;
        }

        std::string printVectorRow(std::vector<double> vector) {
            std::stringstream ss;
            ss << "[" << vector[0];
            for (int i = 1; i < vector.size(); i++) {
                ss << "," << vector[i];
            }
            ss << "]";
            return ss.str();
        }

        std::string printVectorRow(Eigen::Vector3d vector) {
            std::stringstream ss;
            ss << std::fixed;
            ss << "[" << vector.x() << ", " << vector.y() << ", " << vector.z() << "]";
            return ss.str();
        }

        double generateRandomNumber(double lower, double upper, int precision) {
            long randomNumber = random();
            int interval = (int) ((upper - lower) * std::pow(10, precision));
            randomNumber = randomNumber % interval;
            double rescale = (double) randomNumber / std::pow(10., precision);
            return rescale + lower;
        }
    }
}
