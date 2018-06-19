/*
 * MapPoint.cpp
 *
 *  Created on: Jun 13, 2018
 *      Author: sujiwo
 */

#include "MapPoint.h"


using namespace std;
using namespace Eigen;


MapPoint::MapPoint(const Vector3d &p) :
	position(p)
{

}

MapPoint::~MapPoint() {}

