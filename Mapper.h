/*
 * Mapper.h
 *
 *  Created on: Jun 5, 2018
 *      Author: sujiwo
 */

#ifndef MAPPER_H_
#define MAPPER_H_

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


using namespace Eigen;
using std::vector;


struct DataItem {
	std::string imagePath;
	Vector3d position;
	Quaterniond orientation;
};

struct CameraPinholeParamsRead {
	double
		fx, fy,
		cx, cy;
	int width, height;
	Eigen::Matrix<double,3,4> toMatrix() const;
};

typedef std::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > pointCloudPtr;


class Mapper
{
public:

	Mapper(const std::string &datasetDir);
	virtual ~Mapper();

	const CameraPinholeParamsRead& getCameraParams()
	{ return cparams; }

	bool run ();

	void dump (const std::string &filename);

	pointCloudPtr dumpPointCloud ();

private:
	std::vector<DataItem> dataset;
	CameraPinholeParamsRead cparams;

	cv::Mat mask;

//	Feature detector, descriptor and matcher
	cv::Ptr<cv::ORB> featureDetector;
	cv::Ptr<cv::BFMatcher> bfMatch;

	// Map Objects
	vector<KeyFrame*> frameList;
	vector<MapPoint*> pointList;

	// Relationship between MapPoint and KeyFrames
	std::map<MapPoint*, std::set<KeyFrame*> > pointAppearances;
	std::map<KeyFrame*,
		std::map<MapPoint*, uint64> > framePoints;


	void buildKeyFrames();

	cv::Mat vocabulary;
	void trainVocabulary ();
};

#endif /* MAPPER_H_ */
