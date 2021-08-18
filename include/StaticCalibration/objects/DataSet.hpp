//
// Created by brucknem on 18.08.21.
//

#ifndef STATICCALIBRATION_DATASET_HPP
#define STATICCALIBRATION_DATASET_HPP

#include <vector>
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
            std::vector<static_calibration::calibration::WorldObject> worldObjects;

            /**
             * The 2d image objects.
             */
            std::vector<static_calibration::calibration::ImageObject> imageObjects;

            /**
             * The merged combinations of world objects and image objects.
             */
            std::vector<static_calibration::calibration::ParametricPoint> parametricPoints;

            /**
             * Merges the given object with the counterpart with the same id.

             * @tparam T static_calibration::calibration::WorldObject, static_calibration::calibration::ImageObject
             * @param object The object to merge.
             */
            template<class T>
            void merge(const T &object);

            /**
             * Merges the given objects.
             *
             * @param worldObject
             * @param imageObject
             */
            void merge(const calibration::WorldObject &worldObject, const calibration::ImageObject &imageObject);

        public:

            /**
             * @constructor
             */
            explicit DataSet() = default;

            /**
             * @constructor
             */
            DataSet(std::vector<static_calibration::calibration::WorldObject> worldObjects,
                    std::vector<static_calibration::calibration::ImageObject> imageObjects);

            DataSet(const std::string &objectsFile, const std::string &imageObjectsFile);

            /**
             * Generates a dataset from the given file.
             *
             * @tparam T static_calibration::calibration::WorldObject, static_calibration::calibration::ImageObject
             * @param objectsFile The file to load.
             *
             * @return The dataset.
             */
            template<typename T>
            static DataSet from(const std::string &objectsFile);

            /**
             * @get
             */
            const std::vector<static_calibration::calibration::WorldObject> &getWorldObjects() const;

            /**
             * @get
             */
            const std::vector<static_calibration::calibration::ImageObject> &getImageObjects() const;

            /**
             * @get
             */
            const std::vector<static_calibration::calibration::ParametricPoint> &getParametricPoints() const;

            /**
             * Adds an object to the dataset.
             *
             * @tparam T static_calibration::calibration::WorldObject, static_calibration::calibration::ImageObject
             * @param object The object to add.
             */
            template<class T>
            void add(const T &object);

            /**
             * Adds the given objects to the dataset.
             *
             * @param worldObject
             * @param imageObject
             */
            void add(const calibration::WorldObject &worldObject, const calibration::ImageObject &imageObject);

            /**
             * Adds an object to the dataset.
             *
             * @tparam T static_calibration::calibration::WorldObject, static_calibration::calibration::ImageObject
             * @param object The object to add.
             */
            template<class T>
            void loadFromFile(const std::string &objectsFile);

            /**
             * Removes all elements from the dataset.
             */
            void clear();
        };

        template<typename T>
        DataSet DataSet::from(const std::string &objectsFile) {
            DataSet result;
            result.loadFromFile<T>(objectsFile);
            return result;
        }

    }
}


#endif //STATICCALIBRATION_DATASET_HPP
