/*
 * Frame.cpp
 *
 *  Created on: Jul 17, 2018
 *      Author: sujiwo
 */

#include <vector>
#include "Frame.h"
#include "ImageDatabase.h"


using namespace std;
using namespace Eigen;


Frame::	Frame(
	const cv::Mat &imgSrc,
	cv::Ptr<cv::FeatureDetector> fdetector,
	const CameraPinholeParams &cPar,
	const cv::Mat &mask) :

	image(imgSrc),
	_mPos(Vector3d::Zero()),
	_mOri(Quaterniond::Identity())

{
	fdetector->detectAndCompute(image, mask, keypoints, descriptors);
}


Frame::~Frame() {
	// TODO Auto-generated destructor stub
}


void
Frame::computeBoW(const ImageDatabase &idb)
{
	if (words.empty()) {
		vector<cv::Mat> descWrd = toDescriptorVector(descriptors);
		idb.vocabulary().transform(descWrd, words, featureVec, 4);
	}
}
