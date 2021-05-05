//
// Created by brucknem on 13.01.21.
//

#ifndef CAMERASTABILIZATION_IMAGETESTBASE_HPP
#define CAMERASTABILIZATION_IMAGETESTBASE_HPP

#include "gtest/gtest.h"
#include "opencv2/opencv.hpp"
#include <iostream>

namespace static_calibration {
	namespace tests {

		/**
		 * Base class for all toCameraSpace.
		 */
		class ImageTestBase : public ::testing::Test {
		protected:
			/**
			 * The loaded toCameraSpace image.
			 */
			cv::Mat testImgCPU;

			/**
			 * The loaded toCameraSpace image on GPU.
			 */
			cv::cuda::GpuMat testImgGPU;

			/**
			 * Sets up the random number generator for deterministic test and loads the toCameraSpace image.
			 */
			void SetUp() override;

			/**
			 * @destructor
			 */
			~ImageTestBase() override = default;
		};
	}
}

#endif //CAMERASTABILIZATION_IMAGETESTBASE_HPP
