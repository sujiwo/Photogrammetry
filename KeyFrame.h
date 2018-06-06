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
#include <tuple>


typedef std::tuple<theia::ViewId, theia::Feature, theia::ViewId, theia::Feature> FeaturePair;


inline theia::Feature keypoint2feature (const cv::KeyPoint &k)
{ return theia::Feature(k.pt.x, k.pt.y); }


class KeyFrame {
public:
	KeyFrame(const std::string &path,
			const Eigen::Vector3d &p, const Eigen::Quaterniond &o,
			cv::Mat &mask,
			cv::Ptr<cv::FeatureDetector> fdetector,
			theia::Camera &camera0,
			theia::Reconstruction *reconstructor);
	virtual ~KeyFrame();

	inline std::vector<cv::KeyPoint> getKeypoints()
	{ return keypoints; }

	inline cv::Mat getDescriptors()
	{ return descriptors; }

	static void match (const KeyFrame &k1, const KeyFrame &k2,
		cv::Ptr<cv::DescriptorMatcher> matcher,
		std::vector<FeaturePair> &featurePairs
	);

private:
	cv::Mat image;
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat descriptors;

	Eigen::Vector3d position;
	Eigen::Quaterniond orientation;

	theia::ViewId tview;

	KeyFrame* prev;
};

#endif /* KEYFRAME_H_ */
