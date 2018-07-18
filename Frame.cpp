/*
 * Frame.cpp
 *
 *  Created on: Jul 17, 2018
 *      Author: sujiwo
 */

#include "Frame.h"


Frame::	Frame(
	const cv::Mat &imgSrc,
	cv::Ptr<cv::FeatureDetector> fdetector,
	const CameraPinholeParams &cPar,
	const cv::Mat &mask) :
	image(imgSrc)
{
	fdetector->detectAndCompute(image, mask, keypoints, descriptors);
}

Frame::~Frame() {
	// TODO Auto-generated destructor stub
}

