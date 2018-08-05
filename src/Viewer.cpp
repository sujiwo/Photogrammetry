/*
 * Viewer.cpp
 *
 *  Created on: Jul 10, 2018
 *      Author: sujiwo
 */

#include <map>
#include "Viewer.h"
#include "VMap.h"
#include "utilities.h"


using namespace std;


static string wndName ("Viewer");
static cv::Vec3b kpColor(255,0,0);


Viewer::Viewer (const GenericDataset &genset) :
	dataset(genset),
	cMap(NULL)
{
	cv::namedWindow(wndName);
}


Viewer::~Viewer()
{
	cv::destroyWindow(wndName);
}


void
Viewer::setMap (VMap *m)
{ cMap = m; }


vector<cv::KeyPoint>
collectAllKeypoints (const KeyFrame *kf, const vector<kpid> &allKpIds)
{
	vector<cv::KeyPoint> allKp;
	for (auto &i: allKpIds) {
		allKp.push_back(kf->getKeyPointAt(i));
	}
	return allKp;
}


void
Viewer::update(const kfid &frame)
{
	assert (cMap != NULL);

	KeyFrame *kf = cMap->keyframe(frame);
	cv::Mat imagebuf = cv::imread(dataset->at(frame).imagePath);

	// Draw visible map points
	const map<mpid,kpid> visibleMp = cMap->allMapPointsAtKeyFrame(frame);
	if (visibleMp.size() != 0) {
		vector<kpid> allMpKpIds = allValues(visibleMp);
		auto allKeypoints = collectAllKeypoints(kf, allMpKpIds);
		cv::drawKeypoints(imagebuf, allKeypoints, imagebuf, kpColor);
	}

	cv::imshow(wndName, imagebuf);
	cv::waitKey(1);
}
