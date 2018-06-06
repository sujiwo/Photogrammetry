/*
 * KeyFrame.cpp
 *
 *  Created on: Jun 5, 2018
 *      Author: sujiwo
 */

#include "KeyFrame.h"
#include <exception>

using namespace std;


KeyFrame::KeyFrame(const string &path,
		const Eigen::Vector3d &p, const Eigen::Quaterniond &o,
		cv::Mat &mask,
		cv::Ptr<cv::FeatureDetector> fdetector,
		theia::Camera &camera0,
		theia::Reconstruction *reconstructor):
	orientation(o),
	position(p),
	prev(NULL)
{
	image = cv::imread(path, cv::IMREAD_GRAYSCALE);
	if (image.empty()==true)
		throw runtime_error("Unable to open image file");

	tview = reconstructor->AddView(path, 0);

	theia::View *cview = reconstructor->MutableView(0);
	cview->MutableCamera()->SetFromCameraIntrinsicsPriors(camera0.CameraIntrinsicsPriorFromIntrinsics());
	cview->MutableCamera()->SetPosition(position);
	cview->MutableCamera()->SetOrientationFromRotationMatrix(orientation.toRotationMatrix());

	fdetector->detectAndCompute(image, mask, keypoints, descriptors, false);
}

KeyFrame::~KeyFrame() {
	// TODO Auto-generated destructor stub
}


void KeyFrame::match(const KeyFrame &k1, const KeyFrame &k2, cv::Ptr<cv::DescriptorMatcher> matcher)
{
	vector<cv::DMatch> k12matches;
	matcher->match(k1.descriptors, k2.descriptors, k12matches);

	theia::TwoViewInfo tw;
}
