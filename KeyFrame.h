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
#include <boost/serialization/serialization.hpp>

#include "VMap.h"
#include "MapPoint.h"
#include "triangulation.h"


namespace boost {
namespace serialization {
	template <class Archive>
		void serialize (Archive & ar, KeyFrame &keyframe, const unsigned int version);
}
}


//typedef std::tuple<int64, cv::Point2f, int64, cv::Point2f> FeaturePair;
struct FeaturePair {
	kpid kpid1;
	cv::Point2f keypoint1;
	kpid kpid2;
	cv::Point2f keypoint2;
};

struct CameraPinholeParams;


class KeyFrame {
public:

	KeyFrame();

	KeyFrame(const cv::Mat &imgSrc,
			const Eigen::Vector3d &p, const Eigen::Quaterniond &o,
			cv::Mat &mask,
			cv::Ptr<cv::FeatureDetector> fdetector,
			const CameraPinholeParams *cameraIntr);
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
		const KeyFrame *kf1, const KeyFrame *kf2,
		std::vector<kfid> &mapPointList,
		const std::vector<FeaturePair> &featurePairs,
		std::map<mpid, kpid> &mapPointToKeyPointInKeyFrame1,
		std::map<mpid, kpid> &mapPointToKeyPointInKeyFrame2,
		VMap *parent
	);

	kfid getId () const
	{ return id; }

//	const std::vector<MapPoint*>& getVisiblePoints()
//	{ return visiblePoints; }

	Eigen::Matrix<double,3,4> externalParamMatrix () const;
	Eigen::Matrix4d externalParamMatrix4 () const;

	Eigen::Matrix<double,3,4> projectionMatrix () const
	{ return projMatrix; }

	Eigen::Vector2d project (const Eigen::Vector3d &pt3) const;

	Eigen::Vector3d &getPosition ()
	{ return position; }

	Eigen::Quaterniond &getOrientation ()
	{ return orientation; }

//	void appendMapPoint (const MapPoint *mp, uint64 kptIdx);

protected:

	template <class Archive>
    friend void boost::serialization::serialize (Archive & ar, KeyFrame &keyframe, const unsigned int version);


private:
	kfid id;
	cv::Mat image;
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat descriptors;

	Eigen::Vector3d position;
	Eigen::Quaterniond orientation;
	Eigen::Vector3d normal;
	Eigen::Matrix<double,3,4> projMatrix;

	static kfid nextId;

//	KeyFrame* prev;
};

#endif /* KEYFRAME_H_ */
