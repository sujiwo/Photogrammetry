#include <string>
#include <map>
#include <iostream>
#include <sstream>
#include <fstream>
#include <exception>
#include <opencv2/opencv.hpp>
#include "INIReader.h"
#include "KeyFrame.h"
#include "MapBuilder.h"
#include "optimizer.h"



//#include "DataLoader.h"
using namespace std;
using namespace Eigen;

#define Precision 10


string dumpVector(const Vector3d &v)
{
	stringstream s;
	s.precision(Precision);
	s << v.x() << " " << v.y() << " " << v.z();
	return s.str();
}


string dumpVector(const Quaterniond &v)
{
	stringstream s;
	s.precision(Precision);
	s << v.x() << " " << v.y() << " " << v.z() << ' ' << v.w();
	return s.str();
}


void saveCameraPoses (const VMap *mpMap, const string &filename)
{
	fstream fd;
	fd.open(filename, fstream::out|fstream::trunc);
	fd << fixed << setprecision(7);

	auto cameraPoses = mpMap->dumpCameraPoses();
	for (auto &cpose: cameraPoses) {
		Vector3d &position = cpose.first;
		Quaterniond &orientation = cpose.second;

		fd << dumpVector(position) << " " << dumpVector(orientation) << endl;
	}

	fd.close();
}


int main (int argc, char *argv[])
{
	MapBuilder mapBuilder ("/home/sujiwo/Data/track");
	// XXX: Might need to change location

	int maxFrames;
	if (argc < 2)
		maxFrames = 0;
	else
		maxFrames = stoi(string(argv[1]));

	mapBuilder.run2(maxFrames);

//	mapBuilder.dump("/tmp/test_nba.pcd");
//	saveCameraPoses(mapBuilder.getMap(), "/tmp/test_nba.txt");
	mapBuilder.getMap()->save("/home/sujiwo/maptest.map");

	return 0;
}
