#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <vector>
#include <Eigen/Eigen>
#include <unistd.h>
#include <exception>
#include <cstdio>
#include <theia/theia.h>
#include <opencv2/opencv.hpp>
#include "INIReader.h"




//#include "DataLoader.h"

using std::string;
using std::ifstream;
using std::map;
using std::vector;
using namespace Eigen;



//typedef tuple<string, Vector3d, Quaterniond> DataItem;
struct DataItem {
	string imagePath;
	Vector3d position;
	Quaterniond orientation;
};
struct CameraPinholeParamsRead {
	double
		fx, fy,
		cx, cy;
	int width, height;
};

struct KeyFrame {
	cv::Mat image;

};



bool loadDataSet (const string &datadir, vector<DataItem> &dataset, CameraPinholeParamsRead &cparams)
{
	string groundTruthList = datadir + "/pose.txt";
	ifstream inputfd (groundTruthList.c_str());
	if (!inputfd.is_open())
		throw std::runtime_error("Unable to open pose ground truth");

	string cameraParamsFile = datadir + "/camera.txt";
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
		cItem.imagePath = datadir + '/' + std::to_string(iid) + ".png";
		if (access(cItem.imagePath.c_str(), R_OK) != 0)
			throw std::runtime_error("No such file");

		dataset.push_back(cItem);
	}

	return true;
}


void buildKeyFrames (const vector<DataItem> &dataset, vector<KeyFrame> &frameList)
{
	for (auto dataItem: dataset) {
		cv::Mat image = cv::imread(dataItem.imagePath, cv::IMREAD_GRAYSCALE);
	}
}


bool runMapper (const vector<DataItem> &dataset, const CameraPinholeParamsRead &cparams)
{
	// Initialize camera
	theia::Camera camera0 (theia::CameraIntrinsicsModelType::PINHOLE);
//	theia::CameraIntrinsicsPrior camera0intrinsics;
	camera0.SetFocalLength(cparams.fx);
	camera0.SetImageSize(cparams.width, cparams.height);
	camera0.SetPrincipalPoint(cparams.cx, cparams.cy);
//	camera0.Set

	theia::Reconstruction reconstruction;
	theia::ReconstructionEstimatorOptions estOpt;
	estOpt.reconstruction_estimator_type = theia::ReconstructionEstimatorType::INCREMENTAL;
	theia::IncrementalReconstructionEstimator reconstructorEst(estOpt);

	// XXX Unfinished
	theia::View v;
	Eigen::Vector3d zo(0,0,0);
	v.MutableCamera()->SetPosition(zo);

	return true;
}


int main (int argc, char *argv[])
{

	vector<DataItem> imageSet;
	CameraPinholeParamsRead cameraIntrinsicParams;

	// XXX: Might need to change location
	loadDataSet("/home/sujiwo/Works/orb_localizer/test_data/track", imageSet, cameraIntrinsicParams);

	runMapper(imageSet, cameraIntrinsicParams);

	return 0;
}
