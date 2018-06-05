/*
 * KeyFrame.h
 *
 *  Created on: Jun 5, 2018
 *      Author: sujiwo
 */

#ifndef KEYFRAME_H_
#define KEYFRAME_H_

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <theia/theia.h>
#include <vector>
#include <Eigen/Eigen>
#include <string>


class KeyFrame {
public:
	KeyFrame(const std::string &path,
			Eigen::Vector3d p, Eigen::Quaterniond o,
			const cv::Mat &mask,
			const cv::FeatureDetector &fdetector,
			theia::Reconstruction *reconstructor);
	virtual ~KeyFrame();

private:
	cv::Mat image;
	cv::Mat keypoints;
	cv::Mat descriptors;

	Eigen::Vector3d position;
	Eigen::Quaterniond orientation;

	theia::ViewId tview;

	KeyFrame* prev;
};

#endif /* KEYFRAME_H_ */
