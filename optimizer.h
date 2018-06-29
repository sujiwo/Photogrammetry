/*
 * optimizer.h
 *
 *  Created on: Jun 26, 2018
 *      Author: sujiwo
 */

#ifndef OPTIMIZER_H_
#define OPTIMIZER_H_

#include <map>
#include <set>
#include "KeyFrame.h"
#include "MapPoint.h"


void bundle_adjustment (
	std::vector<KeyFrame*> &kfList,
	std::vector<MapPoint*> &mpList,
	std::map<KeyFrame*, std::set<MapPoint*> > kfToMp,
	std::map<MapPoint*, std::set<KeyFrame*> > mpToKf
);


#endif /* OPTIMIZER_H_ */
