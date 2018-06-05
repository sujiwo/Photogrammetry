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
		theia::ViewId i = constructor.AddView(dataItem.imagePath, 0);
		theia::View *cview = constructor.MutableView(i);
		*(cview->MutableCameraIntrinsicsPrior()) = camera0.CameraIntrinsicsPriorFromIntrinsics();
		cview->MutableCamera()->SetPosition(dataItem.position);
		Eigen::Matrix3d rot = dataItem.orientation.matrix();
		cview->MutableCamera()->SetOrientationFromRotationMatrix(rot);


	}
}

