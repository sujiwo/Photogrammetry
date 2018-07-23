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


class Localizer
{
public:

	Localizer(VMap*);

	virtual ~Localizer();

	kfid localize (const cv::Mat &frmImg);

	void setCameraParameterFromId (int cameraId);

protected:
	VMap *sourceMap;

	CameraPinholeParams localizerCamera;
};

#endif /* LOCALIZER_H_ */
