/*
 * Map.h
 *
 *  Created on: Jun 13, 2018
 *      Author: sujiwo
 */

#ifndef MAP_H_
#define MAP_H_

#include <vector>
#include <string>
#include <set>
#include <map>
#include <Eigen/Eigen>

#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"


class KeyFrame;
class MapPoint;


typedef uint64_t oid;


class Map {
public:
	Map();
	virtual ~Map();

	oid createKeyFrame ();
	oid createMapPoint ();

	KeyFrame* getKeyFrameById (const oid &i) const;
	MapPoint* getMapPointById (const oid &i) const;

protected:
	cv::Mat vocabulary;

	std::vector<KeyFrame*> keyframeList;
	std::vector<MapPoint*> mapPointList;

	std::map<oid, KeyFrame*> keyframeInvIdx;
	std::map<oid, MapPoint*> mappointInvIdx;

	// Relationship between MapPoint and KeyFrames
	std::map<MapPoint*, std::set<KeyFrame*> > pointAppearances;
	std::map<KeyFrame*,
		std::map<MapPoint*, oid> > framePoints;

};

#endif /* MAP_H_ */
