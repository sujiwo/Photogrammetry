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


private:
	Eigen::Vector3d position;

	// Best Descriptor
	cv::Mat descriptor;
};

#endif /* MAPPOINT_H_ */