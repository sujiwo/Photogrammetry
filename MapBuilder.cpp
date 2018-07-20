/*
 * Mapper.cpp
 *
 *  Created on: Jun 5, 2018
 *      Author: sujiwo
 */

#include <iostream>
#include <fstream>
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


#define MIN_NEW_POINTS 20
#define MAX_ORB_POINTS_IN_FRAME 6000


typedef Matrix<double,3,4> CameraIntrinsicMatrix;

const Eigen::Vector3d origin(0,0,0);


MapBuilder::MapBuilder(const string &datasetDir) :
	cMap(NULL)
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
	cv::Ptr<cv::ORB> featureDetector = cv::ORB::create(MAX_ORB_POINTS_IN_FRAME);
	cv::Ptr<cv::BFMatcher> bfMatch = cv::BFMatcher::create(cv::NORM_HAMMING2);

	cMap = new VMap(mask, featureDetector, bfMatch);
	cMap->setCameraParameters(cparams);

	viewer = new Viewer(cMap, &dataset);

	inputfd.close();
}


MapBuilder::~MapBuilder()
{ }


void MapBuilder::buildKeyFrames (int startInN, int maxNumOfFrames)
{
	if (maxNumOfFrames==0)
		maxNumOfFrames = dataset.size();

	for (uint i=startInN; i<maxNumOfFrames; i++) {
		createKeyFrame(dataset[i], i);
	}
}


KeyFrame* MapBuilder::createKeyFrame (const DataItem &di, kfid setKfId)
{
	KeyFrame *mNewFrame;
	kfid kfid;
	cv::Mat image = cv::imread(di.imagePath, cv::IMREAD_GRAYSCALE);
	kfid = cMap->createKeyFrame(image, di.position, di.orientation, &mNewFrame, setKfId);
	return mNewFrame;
}


bool MapBuilder::run (int maxKeyframes)
{
	KeyFrame *anchor = createKeyFrame(dataset[0]);

	if (maxKeyframes==0)
		maxKeyframes = dataset.size();

	for (int i=1; i<maxKeyframes; i++) {
		auto &cdi = dataset[i];
		KeyFrame *ckey = createKeyFrame(cdi);

		// Match with anchor
		cMap->estimateStructure(anchor->getId(), ckey->getId());

		anchor = ckey;

		cout << i << '/' << dataset.size() << endl;
	}

	return true;
}


bool MapBuilder::run2 (int startKeyfr, int maxKeyframes)
{
	if (maxKeyframes==0)
		maxKeyframes = dataset.size();
	cout << "Initializing...\n";
	buildKeyFrames(startKeyfr, maxKeyframes);
	vector<kfid> kfList = cMap->getKeyFrameList();

	// Initialize map
	viewer->update(kfList[0]);
	cMap->estimateStructure(kfList[0], kfList[1]);
	cout << "Map initialized\n";
	viewer->update(kfList[1]);

	for (int i=2; i<maxKeyframes; i++) {
		cMap->estimateAndTrack(kfList[i-1], kfList[i]);
		viewer->update(kfList[i]);
		cout << i << '/' << dataset.size() << endl;
	}

	cout << "Bundling...";
	bundle_adjustment(cMap);
	cout << "Done\n";

	cout << "Rebuilding Image DB... ";
	cout.flush();
	cMap->getImageDB()->rebuildAll();
	cout << "Done\n";

	return true;
}


void MapBuilder::dump (const std::string &filename)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr vizCloud =
		cMap->dumpPointCloudFromMapPoints();
	pcl::io::savePCDFileBinary(filename, *vizCloud);
}
//
//
//pointCloudPtr
//MapBuilder::dumpPointCloud ()
//{
//	pointCloudPtr pcv
//		(new pcl::PointCloud<pcl::PointXYZ>(pointList.size(), 1));
//
//	uint i = 0;
//	for (auto *p: pointList) {
//		pcv->at(i).x = p->X();
//		pcv->at(i).y = p->Y();
//		pcv->at(i).z = p->Z();
//		i++;
//	}
//
//	return pcv;
//}
//
//
//void MapBuilder::trainVocabulary()
//{
////	Highly doubtful, this number is too low!
//	cv::BOWKMeansTrainer bowTrainer(MAX_ORB_POINTS_IN_FRAME);
//	cv::Mat sceneDescriptors;
//
//	for (int i=0; i<frameList.size(); i++) {
//		auto *kf = frameList[i];
//
//		for (int j=0; j<kf->numOfKeyPoints(); j++) {
//			bowTrainer.add(kf->getDescriptorAt(j));
//		}
//	}
//
//	vocabulary = bowTrainer.cluster();
//}
