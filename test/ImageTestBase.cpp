//
// Created by brucknem on 13.01.21.
//

#include "ImageTestBase.hpp"

namespace static_calibration {
	namespace tests {
		void ImageTestBase::SetUp() {
			Test::SetUp();
			cv::theRNG().state = 123456789;
			testImgCPU = cv::imread("../misc/test_frame.png");
			testImgGPU.upload(testImgCPU);
		}
	}
}

