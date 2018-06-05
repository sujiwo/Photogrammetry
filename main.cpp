#include <string>
#include <map>
#include <exception>
#include <opencv2/opencv.hpp>
#include "INIReader.h"
#include "KeyFrame.h"
#include "Mapper.h"



//#include "DataLoader.h"

using std::string;
using std::ifstream;
using std::map;
using std::vector;
using namespace Eigen;



//typedef tuple<string, Vector3d, Quaterniond> DataItem;


void buildKeyFrames (const vector<DataItem> &dataset, vector<KeyFrame> &frameList)
{
	auto orbDetector = cv::ORB::create(6000, 1.2, 8);

	// XXX: Unfinished
	for (auto dataItem: dataset) {
		cv::Mat image = cv::imread(dataItem.imagePath, cv::IMREAD_GRAYSCALE);

	}
}


bool runMapper (const vector<DataItem> &dataset, const CameraPinholeParamsRead &cparams)
{
	// Initialize camera
	theia::Camera camera0 (theia::CameraIntrinsicsModelType::PINHOLE);
	theia::CameraIntrinsicsPrior camera0intrinsics;
	camera0.SetFocalLength(cparams.fx);
	camera0.SetImageSize(cparams.width, cparams.height);
	camera0.SetPrincipalPoint(cparams.cx, cparams.cy);
	camera0.SetImageSize(cparams.width, cparams.height);
//	camera0.Set

	theia::Reconstruction reconstruction;
	theia::ReconstructionEstimatorOptions estOpt;
	estOpt.reconstruction_estimator_type = theia::ReconstructionEstimatorType::INCREMENTAL;
	theia::IncrementalReconstructionEstimator reconstructorEst(estOpt);

	// XXX Unfinished
	theia::View v;
	Eigen::Vector3d zo(0,0,0);
	v.MutableCamera()->SetPosition(zo);

	return true;
}


int main (int argc, char *argv[])
{
	Mapper mapBuilder ("/home/sujiwo/Data/track");
	// XXX: Might need to change location
//	loadDataSet("/home/sujiwo/Works/orb_localizer/test_data/track", imageSet, cameraIntrinsicParams);

//	runMapper(imageSet, cameraIntrinsicParams);

	return 0;
}
