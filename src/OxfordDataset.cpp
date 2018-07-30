/*
 * OxfordDataset.cpp
 *
 *  Created on: Jul 28, 2018
 *      Author: sujiwo
 */


#include <cstdio>
#include <Eigen/Eigen>

#include "OxfordDataset.h"


using namespace std;



OxfordDataset::OxfordDataset(const std::string &dirpath, GroundTruthSrc gts) :
	oxPath (dirpath)
{

}


OxfordDataset::~OxfordDataset()
{
}


void
OxfordDataset::loadGps()
{
	const string gpsFilePath = oxPath + "/gps/gps.csv";

}


void
OxfordDataset::loadIns()
{
	const string insFilePath = oxPath + "/gps/ins.csv";
}
