/*
 * Mapper.h
 *
 *  Created on: Jun 5, 2018
 *      Author: sujiwo
 */

#ifndef MAPBUILDER_H_
#define MAPBUILDER_H_

#include <string>
#include <vector>
#include <set>
#include <memory>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "KeyFrame.h"
#include "MapPoint.h"
#include "VMap.h"




struct DataItem {
	std::string imagePath;
	Eigen::Vector3d position;
	Eigen::Quaterniond orientation;
};


typedef std::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > pointCloudPtr;


class MapBuilder
{
public:

	MapBuilder(const std::string &datasetDir);
	virtual ~MapBuilder();

//	const CameraPinholeParamsRead& getCameraParams()
//	{ return cparams; }

	VMap* getMap()
	{ return cMap; }

	bool run (int maxKeyframes=0);
	bool run2 (int maxKeyframes=0);

	void dump (const std::string &filename);

//	pointCloudPtr dumpPointCloud ();

private:
	std::vector<DataItem> dataset;
	CameraPinholeParams cparams;

	VMap *cMap;

	cv::Mat mask;

//	Feature detector, descriptor and matcher
//	cv::Ptr<cv::ORB> featureDetector;
//	cv::Ptr<cv::BFMatcher> bfMatch;

	// Map Objects
//	vector<KeyFrame*> frameList;
//	vector<MapPoint*> pointList;

	void buildKeyFrames();

	cv::Mat vocabulary;
	void trainVocabulary ();

	KeyFrame* createFrame (const DataItem &di);
};

#endif /* MAPBUILDER_H_ */
