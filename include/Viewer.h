/*
 * Viewer.h
 *
 *  Created on: Jul 10, 2018
 *      Author: sujiwo
 */

#ifndef VIEWER_H_
#define VIEWER_H_

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <vector>
#include <thread>
#include "VMap.h"
#include "MapBuilder.h"


class Viewer {
public:
	Viewer (VMap *srMap, const std::vector<DataItem> *ds=NULL);
	~Viewer ();
	void update (const kfid &curFrm);

private:
	VMap *cMap;
	const std::vector<DataItem> *dataset;
};

#endif /* VIEWER_H_ */
