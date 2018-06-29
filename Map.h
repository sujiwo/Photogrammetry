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
#include <Eigen/Eigen>

#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"


class Map {
public:
	Map();
	virtual ~Map();

protected:
	cv::Mat vocabulary;

};

#endif /* MAP_H_ */
