/*
 * Frame.h
 *
 *  Created on: Jul 17, 2018
 *      Author: sujiwo
 */

#ifndef FRAME_H_
#define FRAME_H_


#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "VMap.h"


class Frame
{
public:
	Frame(const cv::Mat &imgSrc,
		cv::Ptr<cv::FeatureDetector> fdetector,
		const CameraPinholeParams &cPar,
		const cv::Mat &mask=cv::Mat());
	virtual ~Frame();

protected:
	cv::Mat image;
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat descriptors;

	Eigen::Vector3d _mPos;
	Eigen::Quaterniond _mOri;
};

#endif /* FRAME_H_ */
