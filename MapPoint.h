/*
 * MapPoint.h
 *
 *  Created on: Jun 13, 2018
 *      Author: sujiwo
 */

#ifndef MAPPOINT_H_
#define MAPPOINT_H_

#include <Eigen/Core>
#include <opencv2/core.hpp>


class MapPoint
{
public:
	MapPoint();
	virtual ~MapPoint();

private:
	Eigen::Vector3d position;

	// Best Descriptor
	cv::Mat descriptor;
};

#endif /* MAPPOINT_H_ */
