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


        std::vector<calibration::WorldObject> loadWorldObjects(const std::string &objectsFile);

        std::vector<calibration::ImageObject> loadImageObjects(const std::string &objectsFile);

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
            DataSet(const std::vector<static_calibration::calibration::WorldObject> &worldObjects,
                    const std::vector<static_calibration::calibration::ImageObject> &imageObjects,
                    const std::map<std::string, std::string> &mapping);

            /**
             * @constructor
             */
            DataSet(const std::string &objectsFile, const std::string &imageObjectsFile,
                    const std::string &mappingFile);

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
            const std::vector<calibration::ParametricPoint> &getParametricPoints() const;

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
            void add(const calibration::WorldObject &worldObject, const calibration::ImageObject &imageObject);

            /**
             * Removes all elements from the dataset.
             */
            void clear();

            void merge(int worldObjectIndex, int imageObjectIndex);
        };


    }
}


#endif //STATICCALIBRATION_DATASET_HPP
