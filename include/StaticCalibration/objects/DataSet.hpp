//
// Created by brucknem on 18.08.21.
//

#ifndef STATICCALIBRATION_DATASET_HPP
#define STATICCALIBRATION_DATASET_HPP

#include <vector>
#include <map>
#include <opencv2/opencv.hpp>
#include <StaticCalibration/camera/RenderingPipeline.hpp>
#include "StaticCalibration/objects/WorldObject.hpp"
#include "StaticCalibration/objects/ImageObject.hpp"

namespace static_calibration {
    namespace objects {


        /**
         * The dataset of 3D world objects and 2D image objects.
         */
        class DataSet {

            /**
             * The 3D world objects.
             */
            std::vector<static_calibration::calibration::Object> worldObjects;

            /**
             * The explicit 3D world road marks.
             */
            std::vector<static_calibration::calibration::RoadMark> explicitRoadMarks;

            /**
             * The 2d image objects.
             */
            std::vector<static_calibration::calibration::ImageObject> imageObjects;

            /**
             * The mapping from 3D world objects to 2D image objects.
             */
            std::map<std::string, std::string> mapping;

            /**
             * Buffer for the parametric points from the mapping of 3D world objects and 2D image objects
             */
            std::vector<calibration::ParametricPoint> worldObjectsParametricPoints;

            /**
             * Buffer for the parametric points from the mapping of 3D world objects and 2D image objects
             */
            std::vector<calibration::ParametricPoint> explicitRoadMarksParametricPoints;

            /**
             * Merges the 3D world objects with the 2D image objects.
             */
            void merge();

        public:

            /**
             * @constructor
             */
            explicit DataSet() = default;

            /**
             * @constructor
             */
            DataSet(std::vector<static_calibration::calibration::Object> worldObjects,
                    std::vector<static_calibration::calibration::RoadMark> explicitRoadMarks,
                    std::vector<static_calibration::calibration::ImageObject> imageObjects,
                    std::map<std::string, std::string> mapping);

            /**
             * @constructor
             */
            DataSet(const std::string &objectsFile, const std::string &explicitRoadMarksFile,
                    const std::string &imageObjectsFile,
                    const std::string &mappingFile);

            /**
             * Generates an extended mapping from road marks to all image objects that are near in image space.
             *
             * @param translation The translation of the camera.
             * @param rotation The rotation of the camera.
             * @param intrinsics The intrinsics of the camera.
             * @param maxDistance The maximum distance in image space of the projected road mark and the image object.
             *
             * @return The mapping from road marks to near image objects.
             */
            std::map<std::string, std::vector<std::string>>
            extendMapping(const Eigen::Vector3d &translation, const Eigen::Vector3d &rotation,
                          const std::vector<double> &intrinsics, int maxDistance);

            /**
             * https://www.geeksforgeeks.org/backtracking-to-find-all-subsets/
             */
            template<class T>
            void generateAllSubsets(std::vector<T> &vector, std::vector<std::vector<T> > &result,
                                    std::vector<T> &subset, int index, int depth, int maxDepth = -1);

            /**
             * https://www.geeksforgeeks.org/backtracking-to-find-all-subsets/
             */
            template<class T>
            std::vector<std::vector<T>> generateAllSubsets(std::vector<T> &vector, int maxDepth = -1);

            /**
             * Creates all possible mappings between image objects and road marks.
             * Be aware that the subset generation is O(n * 2^n), so this is a really slow operation when 'maxElementsPerMapping' is large.
             *
             * @param translation The translation of the camera.
             * @param rotation The rotation of the camera.
             * @param intrinsics The intrinsics of the camera.
             * @param maxDistance The maximum distance in image space of the projected road mark and the image object.
             * @param maxElementsPerMapping The maximal number of elements per mapping.
             *                              Keep this as small as possible as the time complexity for subset generation is O(n * 2^n)
             *
             * @return All possible mappings.
             */
            std::vector<std::map<std::string, std::string>>
            createAllMappings(const Eigen::Vector3d &translation, const Eigen::Vector3d &rotation,
                              const std::vector<double> &intrinsics, int maxDistance, int maxElementsPerMapping = -1);

            /**
             * @get
             */
            template<typename T>
            const std::vector<T> &get() const;

            /**
             * @get
             */
            template<typename T>
            const std::vector<calibration::ParametricPoint> &getParametricPoints() const;

            /**
             * @get
             */
            const std::map<std::string, std::string> &getMapping() const;

            /**
             * Adds an object to the dataset.
             *
             * @tparam T static_calibration::calibration::WorldObject, static_calibration::calibration::ImageObject
             * @param object The object to add.
             */
            template<class T>
            void add(const T &object);

            template<typename T>
            int get(std::string id) const;

            /**
             * Adds the given objects to the dataset.
             *
             * @param worldObject
             * @param imageObject
             */
            template<typename T>
            void add(const T &worldObject, const calibration::ImageObject &imageObject);

            /**
             * Removes all elements from the dataset.
             */
            void clear();

            template<typename T>
            void merge(int worldObjectIndex, int imageObjectIndex);
        };


    }
}


#endif //STATICCALIBRATION_DATASET_HPP
