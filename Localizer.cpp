/*
 * Localizer.cpp
 *
 *  Created on: Jul 23, 2018
 *      Author: sujiwo
 */

#include "Localizer.h"


Localizer::Localizer(VMap *parentMap) :
	sourceMap(parentMap),
	featureDetector(parentMap->getFeatureDetector()),
	imgDb(parentMap->getImageDB())
{}


Localizer::~Localizer()
{}


void
Localizer::setCameraParameterFromId (int cameraId)
{
	localizerCamera = sourceMap->getCameraParameter(cameraId);
}


kfid
Localizer::detect (cv::Mat &frmImg)
{
	Frame frm (frmImg, sourceMap);
	return imgDb->find(frm, true);
}
