/*
 * MapPoint.h
 *
 *  Created on: Jun 13, 2018
 *      Author: sujiwo
 */

#ifndef MAPPOINT_H_
#define MAPPOINT_H_

#include <Eigen/Core>
#include <vector>
#include <tuple>
#include <opencv2/core.hpp>
#include "VMap.h"


class KeyFrame;


struct KeyMapPoint {
	KeyFrame *keyframe;
	int keypointIdx;
};


class MapPoint
{
public:
	MapPoint(const Eigen::Vector3d &p);
	virtual ~MapPoint();

	void createDescriptor(const std::vector<KeyMapPoint> &visibleIn);

	double X() const
	{ return position.x(); }

	double Y() const
	{ return position.y(); }

	double Z() const
	{ return position.z(); }

	Eigen::Vector3d getPosition () const
	{ return position; }

	inline void setPosition (const Eigen::Vector3d &pw)
	{ position = pw; }

	mpid getId () const
	{ return id; }

private:
	Eigen::Vector3d position;

	// Best Descriptor
	cv::Mat descriptor;

	mpid id;

	static mpid nextId;
};

#endif /* MAPPOINT_H_ */
