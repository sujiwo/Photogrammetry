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
#include <memory>
#include <Eigen/Eigen>
#include <theia/theia.h>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

#include "KeyFrame.h"


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
};


class Mapper {
public:
	Mapper(const std::string &datasetDir);
	virtual ~Mapper();

	bool run ();

private:
	std::vector<DataItem> dataset;
	CameraPinholeParamsRead cparams;

	// SfM Object
	theia::Reconstruction *constructor;
	theia::ViewGraph *viewgraph;
	theia::TrackBuilder *trackbuilder;

	cv::Mat mask;

//	Feature detector, descriptor and matcher
	cv::Ptr<cv::ORB> featureDetector;
	cv::Ptr<cv::BFMatcher> bfMatch;

	theia::Camera camera0;

	vector<KeyFrame*> frameList;

	void buildKeyFrames();

};

#endif /* MAPPER_H_ */
