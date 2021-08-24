//
// Created by brucknem on 18.08.21.
//

#ifndef STATICCALIBRATION_DATASET_HPP
#define STATICCALIBRATION_DATASET_HPP

#include <vector>
#include <map>
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
            std::vector<calibration::ParametricPoint> parametricPoints;

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
             * @get
             */
            const std::vector<static_calibration::calibration::Object> &getWorldObjects() const;

            /**
             * @get
             */
            const std::vector<static_calibration::calibration::RoadMark> &getExplicitRoadMarks() const;

            /**
             * @get
             */
            const std::vector<static_calibration::calibration::ImageObject> &getImageObjects() const;

            /**
             * @get
             */
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
