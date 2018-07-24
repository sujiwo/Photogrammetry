/*
 * Localizer.h
 *
 *  Created on: Jul 23, 2018
 *      Author: sujiwo
 */

#ifndef LOCALIZER_H_
#define LOCALIZER_H_


#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

#include "VMap.h"
#include "optimizer.h"
#include "Frame.h"
#include "ImageDatabase.h"


class Localizer
{
public:

	Localizer(VMap*);

	virtual ~Localizer();

	kfid detect (cv::Mat &frmImg);

	void setCameraParameter (const CameraPinholeParams &c)
	{ localizerCamera = c; }

	void setCameraParameterFromId (int cameraId);

protected:
	VMap *sourceMap;
	ImageDatabase *imgDb;

	cv::Ptr<cv::FeatureDetector> featureDetector;

	CameraPinholeParams localizerCamera;
};

#endif /* LOCALIZER_H_ */
