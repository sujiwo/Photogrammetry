/*
 * optimizer.h
 *
 *  Created on: Jun 26, 2018
 *      Author: sujiwo
 */

#ifndef OPTIMIZER_H_
#define OPTIMIZER_H_


#include "KeyFrame.h"
#include "MapPoint.h"


void bundle_adjustment (
	std::vector<KeyFrame*> &kfList,
	std::vector<MapPoint*> &mpList);


#endif /* OPTIMIZER_H_ */
