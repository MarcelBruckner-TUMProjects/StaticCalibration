//
// Created by brucknem on 02.02.21.
//

#ifndef CAMERASTABILIZATION_CAMERAPOSEESTIMATIONBASE_HPP
#define CAMERASTABILIZATION_CAMERAPOSEESTIMATIONBASE_HPP

#include <utility>
#include <vector>
#include <iostream>
#include <thread>
#include <limits>
#include <StaticCalibration/objects/DataSet.hpp>
#include "yaml-cpp/yaml.h"
#include "tinyxml2.h"

#include "ceres/ceres.h"
#include "glog/logging.h"

#include "residuals/CorrespondenceResidual.hpp"
#include "residuals/DistanceFromIntervalResidual.hpp"
#include "residuals/DistanceResidual.hpp"
#include "objects/WorldObject.hpp"

namespace static_calibration {
    namespace calibration {

        /**
         * Prints a vector as a row.
         */
        std::string printVectorRow(Eigen::Vector3d vector);

        /**
         * Prints a vector as a row.
         */
        std::string printVectorRow(std::vector<double> vector);

        /**
         * Generates a random number for the initial guesses within the given interval.
         *
         * @param lower The lower bound of the interval.
         * @param upper The upper bound of the interval.
         *
         * @return A random number in the interval
         */
        static double generateRandomNumber(double lower, double upper, int precision = 3);

        /**
         * Estimates the camera pose from some known correspondences between the world and image.
         *
         * Minimizes the reprojection-error, i.e. x_c = pi(T * R * X_c) for all c in correspondences.
         * X_c are hereby approximated by lines, using the line equation X_c = o_c + lambda_c * h_c
         */
        class CameraPoseEstimationBase {
        private:
            /**
             * The ids of the correspondence residual blocks.
             */
            std::vector<ceres::ResidualBlockId> correspondenceResiduals;

            /**
             * The ids of the correspondence residual blocks.
             */
            std::vector<ceres::ResidualBlockId> explicitRoadMarkResiduals;

            /**
             * The ids of the weight residual blocks.
             */
            std::vector<ceres::ResidualBlockId> weightResiduals;

            /**
             * The ids of the lambda residual blocks.
             */
            std::vector<ceres::ResidualBlockId> lambdaResiduals;

            /**
             * The ids of the rotation residual blocks.
             */
            std::vector<ceres::ResidualBlockId> rotationResiduals;

            /**
             * The final optimization summary.
             */
            ceres::Solver::Summary summary;

            /**
             * The initial camera [x, y, z] translation in world space.
             */
            Eigen::Vector3d initialTranslation;

            /**
             * The initial camera [x, y, z] euler angle rotation around the world space axis.
             */
            Eigen::Vector3d initialRotation;

            /**
             * The dataset of known 3D world objects and 2D image objects.
             */
            static_calibration::objects::DataSet dataSet;

            /**
             * Flag if an initial guess for the rotation was set.
             */
            bool hasRotationGuess = false;

            /**
             * Flag if an initial guess for the translation was set.
             */
            bool hasTranslationGuess = false;

            /**
             * Flag if the optimization is finished, i.e. ceres is finished minimizing.
             */
            bool optimizationFinished = true;

            /**
             * An additional scaling factor for the weight residuals.
             */
            double weightResidualScalingFactor = std::numeric_limits<double>::max();

            /**
             * An additional scaling factor for the lambda residuals.
             */
            double lambdaResidualScalingFactor = 2;

            /**
             * An additional scaling factor for the rotation residuals.
             */
            double rotationResidualScalingFactor = 500;

            /**
             * The distance from the mean of the world objects in z-direction used to estimate an initial guess for
             * the camera translation.
             */
            double initialDistanceFromMean = 500;

            /**
             * True if the found solution after convergence is a valid solution,
             * false if the optimizer didn't converge or converged to a bad minimum.
             */
            bool foundValidSolution = false;

            /**
             * The number of retries in the optimization before the optimizer aborts optimization.
             */
            int maxTriesUntilAbort = 50;

            /**
             * The final loss of the lambda residuals after optimization.
             */
            double lambdasLoss = 0;

            /**
             * The final loss of the correspondence residuals after optimization.
             */
            double correspondencesLoss = 0;

            /**
             * The final loss of the explicit road mark residuals after optimization.
             */
            double explicitRoadMarksLoss = 0;

            /**
             * The final loss of the rotation residuals after optimization.
             */
            double rotationsLoss = 0;

            /**
             * The final loss of the weight residuals after optimization.
             */
            double weightsLoss = 0;

            /**
             * The final loss of all residuals after optimization.
             */
            double totalLoss = 0;


            /**
             * Calculates the mean of the known world correspondences.
             */
            Eigen::Vector3d calculateMean();

            /**
             * Creates a ceres loss function based on the huber loss with additional scaling.
             *
             * @param the Huber loss delta value.
             * @param scale The additional scaling.
             *
             * @return The loss function
             */
            static ceres::ScaledLoss *getScaledHuberLoss(double huber, double scale);

            /**
             * Adds some additional weak constraints on the rotation that guide the optimizer towards useful solutions.
             *
             * - rx in [60, 110]
             * - ry in [-10, 10]
             */
            void addRotationConstraints(ceres::Problem &problem);

            /**
             * Evaluates the problem with the given options.
             *
             * @param problem The ceres problem.
             * @param evalOptions The options, i.e. a lost of residual ids to evaluate only specific residuals.
             *
             * @return The loss of the requested residuals.
             */
            static double evaluate(ceres::Problem &problem,
                                   const ceres::Problem::EvaluateOptions &evalOptions = ceres::Problem::EvaluateOptions());

            /**
             * Evaluates the problem for the correspondence residuals.
             *
             * @param problem The ceres problem.
             *
             * @return The loss of the residuals.
             */
            void evaluateCorrespondenceResiduals(ceres::Problem &problem);

            /**
             * Evaluates the problem for the explicit road marks residuals.
             *
             * @param problem The ceres problem.
             *
             * @return The loss of the residuals.
             */
            void evaluateExplicitRoadMarkResiduals(ceres::Problem &problem);

            /**
             * Evaluates the problem for the weight residuals.
             *
             * @param problem The ceres problem.
             *
             * @return The loss of the residuals.
             */
            void evaluateWeightResiduals(ceres::Problem &problem);

            /**
             * Evaluates the problem for the lambda residuals.
             *
             * @param problem The ceres problem.
             *
             * @return The loss of the residuals.
             */
            void evaluateLambdaResiduals(ceres::Problem &problem);

            /**
             * Evaluates the problem for the rotation residuals.
             *
             * @param problem The ceres problem.
             *
             * @return The loss of the residuals.
             */
            void evaluateRotationResiduals(ceres::Problem &problem);

            /**
             * Evaluates the problem for all residuals.
             *
             * @param problem The ceres problem.
             *
             * @return The loss of the residuals.
             */
            void evaluateAllResiduals(ceres::Problem &problem);

            /**
             * Creates the ceres options used for optimization.
             *
             * @param logSummary Flag to log the ceres summary output to stdout.
             */
            static ceres::Solver::Options setupOptions(bool logSummary);

            /**
             * Adds a correspondence residual block based on the given point to the problem.
             *
             * @param problem The ceres problem
             * @param point The point used in the residual block.
             *
             * @return The residual block id.
             */
            virtual ceres::ResidualBlockId
            addCorrespondenceResidualBlock(ceres::Problem &problem, const ParametricPoint &point,
                                           ceres::LossFunction *lossFunction);

            /**
             * Adds a lambda residual block based on the given point to the problem.
             *
             * @param problem The ceres problem
             * @param point The point used in the residual block.
             * @param lambdaMin The height of the world object.
             *
             * @return The residual block id.
             */
            ceres::ResidualBlockId
            addLambdaResidualBlock(ceres::Problem &problem, const ParametricPoint &point) const;

            /**
             * Adds a lambda residual block based on the given point to the problem.
             *
             * @param problem The ceres problem
             * @param weight The weight to constrain.
             *
             * @return The residual block id.
             */
            ceres::ResidualBlockId addWeightResidualBlock(ceres::Problem &problem, double *weight) const;

        protected:

            /**
             * The final loss of the intrinsic parameter residuals after optimization.
             */
            double intrinsicsLoss = 0;


            /**
             * Evaluates the problem for the residuals of the requested residual blocks.
             *
             * @param problem The ceres problem.
             * @param blockIds The list of residual block ids to evaluate.
             *
             * @return The loss of the requested residuals.
             */
            static double evaluate(ceres::Problem &problem, const std::vector<ceres::ResidualBlockId> &blockIds);

            /**
             * Solves the ceres problem, i.e. runs the optimization and evaluates the remaining losses.
             *
             * @param logSummary Flag to log the ceres summary output to stdout.
             */
            virtual ceres::Problem solveProblem(bool logSummary);

            virtual /**
                 * Creates the ceres problem from the list of known world objects.
                 */
            ceres::Problem createProblem();

            /**
                 * The current camera [x, y, z] translation in world space used for optimization.
                 */
            Eigen::Vector3d translation;
            /**
                 * The current camera [x, y, z] euler angle rotation around the world space axis.
                 */
            Eigen::Vector3d rotation;

            /**
             * The weights of the correspondence residual blocks.
             */
            std::vector<double *> weights;

            /**
             * The current [f_x, ratio, c_x, c_y, skew] intrinsics values of the pinhole camera model.
             */
            std::vector<double> intrinsics;

            /**
             * The initial [f, ratio, c_x, c_y, skew] intrinsics values of the pinhole camera model.
             */
            std::vector<double> initialIntrinsics;

            /**
             * Creates a ceres loss function based on the huber loss with additional scaling.
             *
             * @param scale The additional scaling.
             *
             * @return The loss function
             */
            static ceres::ScaledLoss *getScaledHuberLoss(double scale);

        public:
            /**
             * @constructor
             *
             * @param intrinsics [f_x, ratio, c_x, c_y, skew] The intrinsics of the pinhole camera model.
             */
            explicit CameraPoseEstimationBase(const std::vector<double> &intrinsics);

            /**
             * @destructor
             */
            virtual ~CameraPoseEstimationBase() = default;

            const objects::DataSet &getDataSet() const;

            void setDataSet(const objects::DataSet &dataSet);

            /**
             * Estimates the camera translation and rotation based on the known correspondences between the world and
             * image.
             *
             * @param logSummary Flag to log the ceres summary output to stdout.
             */
            virtual void estimate(bool logSummary);

            /**
             * Async threaded wrapper for the pose estimation function.
             *
             * @param logSummary Flag to log the ceres summary output to stdout.
             */
            std::thread estimateAsync(bool logSummary = false);

            /**
             * Based on the known world positions calculates an initial guess for the camera translation and rotation.
             * This is necessary as the optimization problem is rather ill posed and sensitive to the initialization.
             */
            void calculateInitialGuess();


            /**
             * Brings a rotation vector in the range [-180, 180] for all elements.
             */
            static Eigen::Vector3d clearRotation(const Eigen::Vector3d &rotation);

            /**
             * Set an initial guess for the camera translation in world space.
             * @param translation The guess.
             */
            template<typename T>
            void guessTranslation(const T &translation);

            /**
             * Set an initial guess for the camera rotation in world space.
             * @param translation The guess.
             */
            template<typename T>
            void guessRotation(const T &rotation);

            /**
             * @get
             */
            const Eigen::Vector3d &getTranslation() const;

            /**
             * @get
             */
            Eigen::Vector3d getRotation() const;

            /**
             * @get
             */
            const std::vector<static_calibration::calibration::WorldObject> &getWorldObjects() const;

            /**
             * @return false as long the optimization is running, true else.
             */
            bool isEstimationFinished() const;

            /**
             * @stream
             */
            friend std::ostream &operator<<(std::ostream &os, const CameraPoseEstimationBase &estimator);

            /**
             * @stream
             */
            friend std::ostream &operator<<(std::ostream &os, const CameraPoseEstimationBase *estimator);

            /**
             * @set
             */
            void setWeightPenalizeScale(double weightPenalizeScale);

            /**
             * @get
             */
            std::vector<double> getWeights();

            /**
             * @return true if the found solution is a valid solution based on the rationals on the sizes of the
             * remaining losses, false else.
             */
            bool hasFoundValidSolution() const;

            /**
             * @get
             */
            double getLambdasLoss() const;

            /**
             * @get
             */
            double getCorrespondencesLoss() const;

            /**
             * @get
             */
            double getExplicitRoadMarksLoss() const;

            /**
             * @get
             */
            double getRotationsLoss() const;

            /**
             * @get
             */
            double getWeightsLoss() const;

            /**
             * @get
             */
            double getTotalLoss() const;

            std::vector<double> getLambdas();

            virtual void resetParameters();

            /**
             * @get
             */
            const std::vector<double> &getIntrinsics() const;

            void setIntrinsics(const std::vector<double> &intrinsics);

            /**
             * @get
             */
            double getIntrinsicsLoss() const;

            virtual /**
             * @get
             */
            int getCorrespondenceLossUpperBound() const;

        };
    }
}

#endif //CAMERASTABILIZATION_CAMERAPOSEESTIMATIONBASE_HPP
