/*
 * map_oxford.cpp
 *
 *  Created on: Jul 30, 2018
 *      Author: sujiwo
 */


#include <iostream>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include "OxfordDataset.h"


using namespace std;
using namespace Eigen;


int main (int argc, char *argv[])
{
	OxfordDataset oxf(argv[1], "/home/sujiwo/Sources/robotcar-dataset-sdk/models");
	OxfordDataItem di = oxf.at(1000);
	cv::Mat img = di.getImage();
	cv::imwrite("/tmp/3.png", img);
//	oxf.getCameraParameter("/home/")
	return 0;
}
