/*
 * MapPoint.cpp
 *
 *  Created on: Jun 13, 2018
 *      Author: sujiwo
 */

#include "MapPoint.h"


using namespace std;
using namespace Eigen;


mpid MapPoint::nextId = 0;


MapPoint::MapPoint()
{}


MapPoint::MapPoint(const Vector3d &p) :
	position(p),
	id(nextId++)
{}

MapPoint::~MapPoint() {}

