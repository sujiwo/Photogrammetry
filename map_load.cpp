#include <iostream>
#include <string>
#include <map>
#include <exception>
#include <opencv2/opencv.hpp>
#include "INIReader.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "MapBuilder.h"
#include "optimizer.h"



//#include "DataLoader.h"

using namespace std;
using namespace Eigen;




int main (int argc, char *argv[])
{
	VMap myMap;
	myMap.load(string(argv[1]));

	for (auto mid: myMap.getKeyFrameList()) {
		auto p = myMap.keyframe(mid)->getPosition();
		cout << p.x() << " " << p.y() << " " << p.z() << endl;
	}

	cout << "Done" << endl;
	return 0;
}
