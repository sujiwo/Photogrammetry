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

#include "INIReader.h"
#include "KeyFrame.h"


using namespace std;


Mapper::Mapper(const string &datasetDir) :

	camera0(theia::CameraIntrinsicsModelType::PINHOLE)

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

	// Initialize camera
	theia::CameraIntrinsicsPrior camera0intrinsics;
	camera0.SetFocalLength(cparams.fx);
	camera0.SetImageSize(cparams.width, cparams.height);
	camera0.SetPrincipalPoint(cparams.cx, cparams.cy);
	camera0.SetImageSize(cparams.width, cparams.height);

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
			featureDetector,
			camera0,
			&constructor);
		frameList.push_back(newkf);
	}

	// Build matches
	theia::TrackBuilder trackBuilder (2, dataset.size()*2);
//	KeyFrame *anchor = frameList[0];

	for (int i=1; i<dataset.size(); i++) {
		KeyFrame *kf1 = frameList[i-1],
			*kf2 = frameList[i];

		vector<FeaturePair> featPairs;
		featPairs.clear();

		KeyFrame::match(*kf1, *kf2, bfMatch, featPairs);
		for (auto &fp: featPairs) {
			trackBuilder.AddFeatureCorrespondence(get<0>(fp), get<1>(fp), get<2>(fp), get<3>(fp));
		}
	}

	trackBuilder.BuildTracks(&constructor);
}


bool Mapper::run ()
{
	buildKeyFrames();
	return true;
}
