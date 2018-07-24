/*
 * Mapper.cpp
 *
 *  Created on: Jun 5, 2018
 *      Author: sujiwo
 */

#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <opencv2/highgui.hpp>
#include <pcl/io/pcd_io.h>

#include "INIReader.h"
#include "KeyFrame.h"
#include "MapBuilder.h"
#include "Viewer.h"
#include "optimizer.h"
#include "ImageDatabase.h"


using namespace std;
using namespace Eigen;
using namespace std::chrono;


#define MIN_NEW_POINTS 20


typedef Matrix<double,3,4> CameraIntrinsicMatrix;

const Eigen::Vector3d origin(0,0,0);

static int onlyCamera;


CameraPinholeParams
MapBuilder::loadCameraParamsFromFile(const string &f)
{
	CameraPinholeParams c;
	INIReader cameraParser(f);
	c.fx = cameraParser.GetReal("", "fx", 0);
	c.fy = cameraParser.GetReal("", "fy", 0);
	c.cx = cameraParser.GetReal("", "cx", 0);
	c.cy = cameraParser.GetReal("", "cy", 0);
	c.width = cameraParser.GetInteger("", "width", 0);
	c.height = cameraParser.GetInteger("", "height", 0);

	return c;
}


MapBuilder::MapBuilder(const string &datasetDir) :
	cMap(NULL),
	runBADB(true)
{
	string groundTruthList = datasetDir + "/pose.txt";
	ifstream inputfd (groundTruthList.c_str());
	if (!inputfd.is_open())
		throw std::runtime_error("Unable to open pose ground truth");

	string cameraParamsFile = datasetDir + "/camera.txt";
	cparams = MapBuilder::loadCameraParamsFromFile(cameraParamsFile);

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
			throw std::runtime_error(string("No such file: ")+cItem.imagePath);

		dataset.push_back(cItem);
	}

	mask = cv::imread(datasetDir+"/mask.png", cv::IMREAD_GRAYSCALE);
	cMap = new VMap(mask, FeatureDetectorT::ORB, DescriptorMatcherT::BruteForce);
	onlyCamera = cMap->addCameraParameter(cparams);

	viewer = new Viewer(cMap, &dataset);

	inputfd.close();
}


MapBuilder::~MapBuilder()
{
	delete viewer;
}


void MapBuilder::buildKeyFrames (int startInN, int maxNumOfFrames)
{
	if (maxNumOfFrames==0 or startInN + maxNumOfFrames > dataset.size())
		maxNumOfFrames = dataset.size();

#pragma omp parallel
	for (uint i=startInN; i<startInN + maxNumOfFrames; i++) {
		cerr << "ID: " << i << endl;
		createKeyFrame(dataset[i], i);
	}
}


KeyFrame* MapBuilder::createKeyFrame (const DataItem &di, kfid setKfId)
{
	KeyFrame *mNewFrame;
	kfid kfid;
	cv::Mat image = cv::imread(di.imagePath, cv::IMREAD_GRAYSCALE);
	kfid = cMap->createKeyFrame(image, di.position, di.orientation, onlyCamera, &mNewFrame, setKfId);
	return mNewFrame;
}


bool MapBuilder::run2 (int startKeyfr, int maxNumOfKeyframes)
{
	if (maxNumOfKeyframes==0)
		maxNumOfKeyframes = dataset.size();
	cout << "Initializing...\n";
	buildKeyFrames(startKeyfr, maxNumOfKeyframes);
	vector<kfid> kfList = cMap->getKeyFrameList();

	// Initialize map
	viewer->update(kfList[0]);
	cMap->estimateStructure(kfList[0], kfList[1]);
	cout << "Map initialized\n";
	viewer->update(kfList[1]);

	for (int i=2; i<kfList.size(); i++) {
		kfid fromKfId = kfList[i-1],
			toKfId = kfList[i];
		cMap->estimateAndTrack(fromKfId, toKfId);
		viewer->update(toKfId);
		cout << i << '/' << kfList.size() << endl;
	}

	system_clock::time_point t1 = system_clock::now();

	if (runBADB) {

		thread ba([this] {
					cout << "Bundling...";
					bundle_adjustment(cMap);
					cout << "Done\n";
		});

		thread db([this] {
					cout << "Rebuilding Image DB... ";
					cout.flush();
					cMap->getImageDB()->rebuildAll();
					cout << "Done\n";
		});

		ba.join();
		db.join();
	}

	system_clock::time_point t2 = system_clock::now();
	duration<float> td = t2 - t1;
	cerr << "Time(s): " << td.count() << endl;

	return true;
}


void MapBuilder::dump (const std::string &filename)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr vizCloud =
		cMap->dumpPointCloudFromMapPoints();
	pcl::io::savePCDFileBinary(filename, *vizCloud);
}
