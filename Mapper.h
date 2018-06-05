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
#include <Eigen/Eigen>
#include <theia/theia.h>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>


using namespace Eigen;


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

private:
	std::vector<DataItem> dataset;
	CameraPinholeParamsRead cparams;
	theia::Reconstruction constructor;

	cv::Mat mask;
	cv::Ptr<cv::ORB> featureDetector;

	theia::Camera camera0;

	void buildKeyFrames();

};

#endif /* MAPPER_H_ */
