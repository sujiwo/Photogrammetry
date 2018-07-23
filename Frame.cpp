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
	cv::Mat &imgSrc,
	const VMap* parent
) :

	image(imgSrc),
	_mPos(Vector3d::Zero()),
	_mOri(Quaterniond::Identity())

{
	parent->getFeatureDetector()->detectAndCompute(
		image,
		parent->getMask(),
		keypoints,
		descriptors);
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
