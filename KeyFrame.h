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
#include <vector>
#include <map>
#include <Eigen/Eigen>
#include <string>
#include <memory>
#include <tuple>

#include "MapPoint.h"
#include "triangulation.h"



//typedef std::tuple<int64, cv::Point2f, int64, cv::Point2f> FeaturePair;
struct FeaturePair {
	uint64 id1;
	cv::Point2f keypoint1;
	uint64 id2;
	cv::Point2f keypoint2;
};

struct CameraPinholeParamsRead;


class KeyFrame {
public:
	KeyFrame(const std::string &path,
			const Eigen::Vector3d &p, const Eigen::Quaterniond &o,
			cv::Mat &mask,
			cv::Ptr<cv::FeatureDetector> fdetector,
			const CameraPinholeParamsRead *cameraIntr);
	virtual ~KeyFrame();

	inline int numOfKeyPoints() const
	{ return keypoints.size(); }

	inline std::vector<cv::KeyPoint> getKeypoints() const
	{ return keypoints; }

	inline cv::KeyPoint getKeyPointAt (int idx) const
	{ return keypoints[idx]; }

	inline cv::Mat getDescriptors() const
	{ return descriptors; }

	inline cv::Mat getDescriptorAt(int idx) const
	{ return descriptors.row(idx).clone(); }

	static void match (const KeyFrame &k1, const KeyFrame &k2,
		cv::Ptr<cv::DescriptorMatcher> matcher,
		std::vector<FeaturePair> &featurePairs
	);

	static void triangulate (
		KeyFrame &kf1, KeyFrame &kf2,
		std::vector<MapPoint*> &ptsList,
		const std::vector<FeaturePair> &featurePairs
	);

	uint64 getId () const
	{ return id; }

//	const std::vector<MapPoint*>& getVisiblePoints()
//	{ return visiblePoints; }

	Eigen::Matrix<double,3,4> externalParamMatrix () const;
	Eigen::Matrix4d externalParamMatrix4 () const;

	Eigen::Matrix<double,3,4> projectionMatrix () const
	{ return projMatrix; }

	Eigen::Vector2d project (const Eigen::Vector3d &pt3) const;

	Eigen::Vector3d getPosition () const
	{ return position; }

	Eigen::Quaterniond getOrientation () const
	{ return orientation; }

//	void appendMapPoint (const MapPoint *mp, uint64 kptIdx);

private:
	uint64 id;
	cv::Mat image;
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat descriptors;

	Eigen::Vector3d position;
	Eigen::Quaterniond orientation;
	Eigen::Vector3d normal;
	Eigen::Matrix<double,3,4> projMatrix;

	// Visibility information
//	std::vector<MapPoint*> visiblePoints;
//	std::map<const uint64, uint64> mapPointIdx;

	static uint64 nextId;

	KeyFrame* prev;
};

#endif /* KEYFRAME_H_ */
