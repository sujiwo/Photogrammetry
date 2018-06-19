/*
 * Mapper.cpp
 *
 *  Created on: Jun 5, 2018
 *      Author: sujiwo
 */

#include "Mapper.h"
#include <iostream>
#include <fstream>
#include <opencv2/highgui.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

#include "INIReader.h"
#include "KeyFrame.h"


using namespace std;
using namespace Eigen;


#define MIN_NEW_POINTS 20


Mapper::Mapper(const string &datasetDir)
{
	string groundTruthList = datasetDir + "/pose.txt";
	ifstream inputfd (groundTruthList.c_str());
	if (!inputfd.is_open())
		throw std::runtime_error("Unable to open pose ground truth");

	string cameraParamsFile = datasetDir + "/camera.txt";
	INIReader cameraParser(cameraParamsFile);
	cparams.fx = cameraParser.GetReal("", "fx", 0);
	cparams.fy = cameraParser.GetReal("", "fy", 0);
	cparams.cx = cameraParser.GetReal("", "cx", 0);
	cparams.cy = cameraParser.GetReal("", "cy", 0);
	cparams.width = cameraParser.GetInteger("", "width", 0);
	cparams.height = cameraParser.GetInteger("", "height", 0);

	string line;
	while (true) {
		if (!getline(inputfd, line))
			break;

		DataItem cItem;
		int iid;
		float timestamp;
		double x, y, z, qx, qy, qz, qw;
		sscanf (line.c_str(), "%d %f %lf %lf %lf %lf %lf %lf %lf", &iid, &timestamp,
			&x, &y, &z,
			&qx, &qy, &qz, &qw);
		cItem.position << x, y, z;
		cItem.orientation.x() = qx;
		cItem.orientation.y() = qy;
		cItem.orientation.z() = qz;
		cItem.orientation.w() = qw;
		cItem.imagePath = datasetDir + '/' + std::to_string(iid) + ".png";
		if (access(cItem.imagePath.c_str(), R_OK) != 0)
			throw std::runtime_error("No such file");

		dataset.push_back(cItem);
	}

	mask = cv::imread(datasetDir+"/mask.png", cv::IMREAD_GRAYSCALE);
	featureDetector = cv::ORB::create(6000);
	bfMatch = cv::BFMatcher::create(cv::NORM_HAMMING2);

	inputfd.close();
}


Mapper::~Mapper() {
	// TODO Auto-generated destructor stub
}


void Mapper::buildKeyFrames ()
{
	for (auto &dataItem: dataset) {
		KeyFrame *newkf = new KeyFrame(dataItem.imagePath,
			dataItem.position, dataItem.orientation,
			mask,
			featureDetector);
		frameList.push_back(newkf);
	}
}


bool Mapper::run ()
{
	// First keyframe
	KeyFrame *anchor = new KeyFrame (dataset[0].imagePath, dataset[0].position, dataset[0].orientation, mask, featureDetector);
	frameList.push_back(anchor);

	for (int i=1; i<dataset.size(); i++) {
		auto &cdi = dataset[0];
		KeyFrame *ckey = new KeyFrame (cdi.imagePath, cdi.position, cdi.orientation, mask, featureDetector);

		// Match with anchor
		vector<FeaturePair> match12;
		vector<MapPoint*> newMapPoints;
		KeyFrame::match(*anchor, *ckey, bfMatch, match12);
		KeyFrame::triangulate(*anchor, *ckey, newMapPoints, match12);

		if (newMapPoints.size() < MIN_NEW_POINTS) {
			// Switch the anchor
			anchor = ckey;
		}

		frameList.push_back(ckey);
		pointList.insert(pointList.end(), newMapPoints.begin(), newMapPoints.end());

		// What now ?
	}

	return true;
}


void Mapper::dump (const std::string &filename)
{
	pcl::PointCloud<pcl::PointXYZ> vizCloud;
	vizCloud.width = pointList.size();
	vizCloud.height = 1;
	vizCloud.resize(vizCloud.width);

	uint i = 0;
	for (auto *p: pointList) {
		vizCloud.at(i).x = p->X();
		vizCloud.at(i).y = p->Y();
		vizCloud.at(i).z = p->Z();
		i++;
	}

	pcl::io::savePCDFileBinary(filename, vizCloud);
}


Matrix<double,3,4> CameraPinholeParamsRead::toMatrix()
{
	Matrix<double,3,4> K = Matrix<double,3,4>::Zero();
	K(2,2) = 1.0;
    K(0,0) = fx;
    K(1,1) = fy;
    K(0,2) = cx;
    K(1,2) = cy;
	return K;
}
