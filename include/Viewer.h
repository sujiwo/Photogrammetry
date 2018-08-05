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
#include "GenericDataset.h"



class Viewer {
public:

	Viewer (const GenericDataset &genset);
	~Viewer ();

	void update (const uint64_t &dataItemId);

	void setMap (VMap *m);

private:
	const GenericDataset &dataset;
	VMap *cMap;
};

#endif /* VIEWER_H_ */
