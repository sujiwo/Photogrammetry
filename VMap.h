/*
 * Map.h
 *
 *  Created on: Jun 13, 2018
 *      Author: sujiwo
 */

#ifndef VMAP_H_
#define VMAP_H_

#include <vector>
#include <string>
#include <set>
#include <map>
#include <Eigen/Eigen>

#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"


class KeyFrame;
class MapPoint;


typedef uint64_t kfid;
typedef uint64_t mpid;
typedef int kpid;


struct CameraPinholeParams {
	double
		fx, fy,
		cx, cy;
	int width, height;
	Eigen::Matrix<double,3,4> toMatrix() const;
};


class VMap {
public:
	VMap (const cv::Mat &mask, cv::Ptr<cv::FeatureDetector> fdetect, cv::Ptr<cv::DescriptorMatcher> dmatcher);
	virtual ~VMap();

	inline void setCameraParameters (const CameraPinholeParams &vscamIntr)
	{ camera = vscamIntr; }

	inline const CameraPinholeParams getCameraParameters()
	{ return camera; }

	kfid createKeyFrame (
		const cv::Mat &imgSrc,
		const Eigen::Vector3d &p, const Eigen::Quaterniond &o,
		KeyFrame **ptr=NULL);

	mpid createMapPoint (
		const Eigen::Vector3d &p,
		MapPoint **ptr=NULL);

	KeyFrame* getKeyFrameById (const kfid &i) const;
	MapPoint* getMapPointById (const mpid &i) const;

	KeyFrame* keyframe (const kfid &i)
	{ return keyframeInvIdx.at(i); }

	MapPoint* mappoint (const mpid &i)
	{ return mappointInvIdx.at(i); }

	void estimateStructure (const kfid &keyFrame1, const kfid &keyFrame2);

	std::vector<kfid> allKeyFrames () const;
	std::vector<mpid> allMapPoints () const;

	std::set<kfid> getRelatedKeyFrames (const mpid &i) const
	{ return pointAppearances.at(i); }

	inline kpid getKeyPointId (const kfid k, const mpid p)
	{ return framePoints.at(k).at(p); }

protected:
	cv::Mat vocabulary;

//	std::vector<KeyFrame*> keyframeList;
//	std::vector<MapPoint*> mapPointList;

	std::map<kfid, KeyFrame*> keyframeInvIdx;
	std::map<mpid, MapPoint*> mappointInvIdx;

	// Relationship between MapPoint and KeyFrames

	// Maps MapPoint to set of KeyFrames in which it appears
	std::map<mpid, std::set<kfid> > pointAppearances;

	// List all map points that appears in a keyframe with
	// their ID of KeyPoint (ie. their projections)
	// KeyFrame -> (MapPoint, KeyPoint)
	std::map<kfid,
		std::map<mpid, kpid> > framePoints;

	// 2D image stuff
	cv::Mat mask;
	cv::Ptr<cv::FeatureDetector> featureDetector;
	cv::Ptr<cv::DescriptorMatcher> descriptorMatcher;
	CameraPinholeParams camera;
};

#endif /* VMAP_H_ */
