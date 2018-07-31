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
	OxfordDataset oxf(argv[1]);
	oxf.dumpGroundTruth();
	return 0;
}
