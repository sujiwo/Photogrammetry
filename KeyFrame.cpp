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
		Eigen::Vector3d p, Eigen::Quaterniond o,
		const cv::Mat &mask,
		const cv::FeatureDetector &fdetector,
		theia::Reconstruction *reconstructor):
	orientation(o),
	position(p)
{
	image = cv::imread(path, cv::IMREAD_GRAYSCALE);
	if (image.empty()==true)
		throw runtime_error("Unable to open image file");
//	tview = reconstructor->AddView(view_name, group_id)
}

KeyFrame::~KeyFrame() {
	// TODO Auto-generated destructor stub
}

