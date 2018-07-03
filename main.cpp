#include <string>
#include <map>
#include <exception>
#include <opencv2/opencv.hpp>
#include "INIReader.h"
#include "KeyFrame.h"
#include "MapBuilder.h"



//#include "DataLoader.h"

using std::string;
using std::ifstream;
using std::map;
using std::vector;
using namespace Eigen;




int main (int argc, char *argv[])
{
	MapBuilder mapBuilder ("/home/sujiwo/Data/track");
	// XXX: Might need to change location
	mapBuilder.run();
	mapBuilder.dump("/tmp/test.pcd");

	return 0;
}
