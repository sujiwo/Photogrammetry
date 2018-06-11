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
		theia::Reconstruction* reconstructor):
	orientation(o),
	position(p),
	prev(NULL)
{
	image = cv::imread(path, cv::IMREAD_GRAYSCALE);
	if (image.empty()==true)
		throw runtime_error("Unable to open image file");

	tview = reconstructor->AddView(path, 0);
	theia::View *cview = reconstructor->MutableView(tview);
	cview->MutableCamera()->SetFromCameraIntrinsicsPriors(camera0.CameraIntrinsicsPriorFromIntrinsics());
	cview->MutableCamera()->SetPosition(position);
	cview->MutableCamera()->SetOrientationFromRotationMatrix(orientation.toRotationMatrix());

	fdetector->detectAndCompute(image, mask, keypoints, descriptors, false);
	camera = cview->MutableCamera();
}

KeyFrame::~KeyFrame()
{
	// TODO Auto-generated destructor stub
}


void KeyFrame::match(const KeyFrame &k1, const KeyFrame &k2,
	cv::Ptr<cv::DescriptorMatcher> matcher,
	vector<FeaturePair> &featurePairs
)
{
	vector<cv::DMatch> k12matches;
	matcher->match(k1.descriptors, k2.descriptors, k12matches);

	for (auto &m: k12matches) {
		if (m.trainIdx < k1.keypoints.size() and m.queryIdx < k2.keypoints.size()) {
			theia::Feature fk1 = keypoint2feature(k1.keypoints[m.queryIdx]);
			theia::Feature fk2 = keypoint2feature(k2.keypoints[m.trainIdx]);
			featurePairs.push_back (make_tuple(k1.tview, fk1, k2.tview, fk2));
		}
	}
}
