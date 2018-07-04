#include <iostream>
#include <string>
#include <map>
#include <exception>
#include <opencv2/opencv.hpp>
#include "INIReader.h"
#include "KeyFrame.h"
#include "MapBuilder.h"
#include "optimizer.h"



//#include "DataLoader.h"

using namespace std;
using namespace Eigen;




int main (int argc, char *argv[])
{
	VMap myMap;
	myMap.load("/home/sujiwo/maptest.map");

	cout << "Done" << endl;

		cout << "Bundling..." << endl;
		bundle_adjustment (&myMap);

	return 0;
}
