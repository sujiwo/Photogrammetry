/*
 * OxfordDataset.cpp
 *
 *  Created on: Jul 28, 2018
 *      Author: sujiwo
 */


#include <fstream>
#include <cstdio>
#include <Eigen/Eigen>
#include <opencv2/highgui.hpp>
#include <yaml-cpp/yaml.h>

#include "csv.h"
#include "OxfordDataset.h"
#include "utilities.h"


using namespace std;
using namespace Eigen;


static const set<int> GpsColumns ({0,9,8,4,2,3});
static const set<int> InsColumns ({0,6,5,4,12,13,14,10,9,11,2,3});


/*
 * Transformation from INS to stereo camera
 * Might be inaccurate
 */
static const TTransform
baseLinkToOffset = TTransform::from_Pos_Quat(
	Vector3d (0.0, 1.720, 1.070),
	TQuaternion (-0.723, 0.007, 0.002, 0.691));



OxfordDataset::OxfordDataset(

	const std::string &dirpath,
	const std::string &modelDir,
	GroundTruthSrc gts) :

	oxPath (dirpath)
{
	oxfCamera.fx = -1;
	loadTimestamps();
	loadGps();
	loadIns();

	loadModel(modelDir);

	createStereoGroundTruths();
	return;
}


OxfordDataset::~OxfordDataset()
{
}


void
OxfordDataset::loadGps()
{
	const string gpsFilePath = oxPath + "/gps/gps.csv";
	StringTable GPS_s = create_table(gpsFilePath, GpsColumns, true);
	const size_t ss = GPS_s.size();
	gpsPoseTable.resize(ss);
//	gpsTimestamps.resize(ss);

	for (uint i=0; i<ss; i++) {
		GpsPose ps;
			ps.timestamp = stoul(GPS_s.get(i, "timestamp"));
			ps.easting = stod(GPS_s.get(i, "easting"));
			ps.northing = stod(GPS_s.get(i, "northing"));
			ps.altitude = stod(GPS_s.get(i, "altitude"));
			ps.latitude = stod(GPS_s.get(i, "latitude"));
			ps.longitude = stod(GPS_s.get(i, "longitude"));
		gpsPoseTable[i] = ps;
	}
}


//CameraPinholeParams
//OxfordDataset::getCameraParameter(const std::string &yamlFileCameraParams)
//{
//	if (yamlFileCameraParams.size()==0) {
//		if (oxfCamera.fx<0)
//			throw runtime_error("Camera parameter has not been loaded");
//		else
//			return oxfCamera;
//	}
//
//	YAML::Node camCfg = YAML::LoadFile(yamlFileCameraParams);
//	oxfCamera.width = camCfg["image_width"].as<int>();
//	oxfCamera.height = camCfg["image_height"].as<int>();
//	// TODO: Read fx/cx/fy/cy
//	vector<double> intrinsics = camCfg["camera_matrix"]["data"].as<vector<double>>();
//	oxfCamera.fx = intrinsics[0];
//	oxfCamera.cx = intrinsics[2];
//	oxfCamera.fy = intrinsics[4];
//	oxfCamera.cy = intrinsics[5];
//
//	return oxfCamera;
//}


void
OxfordDataset::loadIns()
{
	const string insFilePath = oxPath + "/gps/ins.csv";
	StringTable INS_s = create_table(insFilePath, InsColumns, true);
	const size_t ss = INS_s.size();
	insPoseTable.resize(ss);
//	insTimestamps.resize(ss);

	for (uint i=0; i<ss; i++) {
		InsPose is;
			is.timestamp = stoul(INS_s.get(i, "timestamp"));
			is.easting = stod(INS_s.get(i, "easting"));
			is.northing = stod(INS_s.get(i, "northing"));
			is.altitude = stod(INS_s.get(i, "altitude"));
			is.latitude = stod(INS_s.get(i, "latitude"));
			is.longitude = stod(INS_s.get(i, "longitude"));
			is.roll = stod(INS_s.get(i, "roll"));
			// Correct pitch & yaw rotations to more sensible ROS convention;
			// otherwise you will have problems later
			is.pitch = -stod(INS_s.get(i, "pitch"));
			is.yaw = -stod(INS_s.get(i, "yaw"));

		insPoseTable[i] = is;
	}
}


void OxfordDataset::loadTimestamps()
{
	const string timestampsPath = oxPath + "/stereo.timestamps";
	StringTable TS = create_table(timestampsPath);
	const size_t ss = TS.size();
	stereoTimestamps.resize(ss);
	stereoImagePaths.resize(ss);

	for (uint32_t i=0; i<ss; i++) {
		const string &tsstr = TS.get(i,0);
		stereoTimestamps[i] = stoul(tsstr);

		const string
			ctrname = oxPath + "/stereo/centre",
        	lftname = oxPath + "/stereo/left",
			rhtname = oxPath + "/stereo/right";

		stereoImagePaths[i][StereoImagePath::LEFT] = lftname + '/' + tsstr + ".png";
		stereoImagePaths[i][StereoImagePath::CENTER] = ctrname + '/' + tsstr + ".png";
		stereoImagePaths[i][StereoImagePath::RIGHT] = rhtname + '/' + tsstr + ".png";
	}
}


Pose fromINS(const InsPose &ps)
{
	return TTransform::from_XYZ_RPY(
		Vector3d(
			ps.easting + OriginCorrectionEasting,
			ps.northing + OriginCorrectionNorthing,
			ps.altitude),
		ps.roll, ps.pitch, ps.yaw
	);
}


Pose interpolateFromINS (
	uint64_t timestamp,
	const InsPose &ps1,
	const InsPose &ps2)
{
	assert(timestamp >= ps1.timestamp and timestamp<=ps2.timestamp);

	Pose px1 = fromINS(ps1),
		px2 = fromINS(ps2);

	double ratio = double(timestamp - ps1.timestamp) / double(ps2.timestamp - ps1.timestamp);
	Vector3d px_pos = px1.position() + ratio * (px2.position() - px1.position());
	Quaterniond px_or = px1.orientation().slerp(ratio, px2.orientation());
	px_or.normalize();

	return Pose::from_Pos_Quat(px_pos, px_or);
}


void
OxfordDataset::createStereoGroundTruths()
{
	map<uint64_t,InsPose*> tsFinder;

	// 1: create tree for traversing timestamp data
	for (uint32_t i=0; i<insPoseTable.size(); i++) {
		InsPose *p = &insPoseTable.at(i);
		tsFinder.insert(make_pair(insPoseTable[i].timestamp, p));
		if (i>0 and p->timestamp <= (p-1)->timestamp)
			throw range_error("Invalid decreasing timestamp detected");
	}

	// 2
	auto Itx = tsFinder.begin();
	for (uint32_t i=0; i<stereoTimestamps.size(); i++) {

		uint64_t ts = stereoTimestamps[i];
		Pose px;

		if (ts < insPoseTable[0].timestamp) {
			px = fromINS(insPoseTable[0]);
		}

		else if (ts > insPoseTable[insPoseTable.size()-1].timestamp) {
			px = fromINS(insPoseTable[insPoseTable.size()-1]);
		}

		else {
			decltype(Itx) Itx_prev;
			do {
				Itx_prev = Itx;
				++Itx;
			} while (ts > (*Itx).first and Itx!=tsFinder.end());

			const uint64_t ts1 = (*Itx_prev).first,
				ts2 = (*Itx).first;
			const InsPose& ps1 = *tsFinder[ts1],
				&ps2 = *tsFinder[ts2];

			px = interpolateFromINS(ts, ps1, ps2);
		}

		// Transform INS/baselink position to camera
		px = px * baseLinkToOffset;

		stereoGroundTruths.insert(make_pair(ts, px));
	}
}


void OxfordDataset::dumpGroundTruth(const string &fp)
{
	const char dumpSep = ',';
	ostream *fd;
	ofstream fdr;
	if (fp.size()==0)
		fd = &cout;
	else {
		fdr.open(fp);
		if (fdr.good())
			throw runtime_error("Unable to open file");
		fd = &fdr;
	}

	*fd << fixed;
	*fd << setprecision(4);

	for (int i=0; i<stereoTimestamps.size(); i++) {
		auto t = stereoTimestamps.at(i);
		float tss = (float)t / 1e6;
		auto trans = stereoGroundTruths.at(t);
		Vector3d rpy = quaternionToRPY(trans.orientation());
		*fd << tss
			<< dumpSep << trans.position().x()
			<< dumpSep << trans.position().y()
			<< dumpSep << trans.position().z()
			<< dumpSep << rpy[0]
			<< dumpSep << rpy[1]
			<< dumpSep << rpy[2]
			<< endl;
	}
}


OxfordDataItem
OxfordDataset::at(const int i)
{
	OxfordDataItem di;
	di.parent = this;
	di.timestamp = stereoTimestamps[i];
	di.paths = stereoImagePaths[i];
	di.groundTruth = stereoGroundTruths.at(di.timestamp);
	return di;
}


cv::Mat
OxfordDataItem::getImage(int which)
{
	const string &path = this->paths[which];
	cv::Mat img = cv::imread(path, cv::IMREAD_GRAYSCALE);

	// XXX: need better demosaicing algorithms
	cv::cvtColor(img, img, CV_BayerGB2RGB);
	img = parent->undistort(img);

	return img;
}


void
OxfordDataset::loadModel(const string &modelDir)
{
	string
		centerLut = modelDir + "/stereo_narrow_left_distortion_lut.bin",
		centerIntrinsic = modelDir + "/stereo_narrow_left.txt";

	// LUT distortion correction table
	std::ifstream lutfd (centerLut, ifstream::ate|ifstream::binary);
	const size_t lutfdsize = lutfd.tellg();
	if (lutfdsize%sizeof(double) != 0)
		throw runtime_error("File size is not correct");
	lutfd.seekg(ifstream::beg);

	cv::Mat distortionLUT_center (2, lutfdsize/(sizeof(double)*2), CV_64F);
	lutfd.read((char*)distortionLUT_center.ptr(0), lutfdsize/2);
	lutfd.read((char*)distortionLUT_center.ptr(1), lutfdsize/2);
	distortionLUT_center.row(0).convertTo(distortionLUT_center_x, CV_32F);
	distortionLUT_center.row(1).convertTo(distortionLUT_center_y, CV_32F);

	// Camera intrinsic parameters
	StringTable intr = create_table(centerIntrinsic);
	oxfCamera.fx = stod(intr.get(0,0));
	oxfCamera.fy = stod(intr.get(0,1));
	oxfCamera.cx = stod(intr.get(0,2));
	oxfCamera.cy = stod(intr.get(0,3));
	oxfCamera.width = oxfCamera.height = -1;
	// XXX: image width & height have not been loaded
}


cv::Mat
OxfordDataset::undistort (cv::Mat &src)
{
	// Hint: use cv::remap
	if (oxfCamera.width==-1) {
		oxfCamera.width=src.cols;
		oxfCamera.height=src.rows;

		if (oxfCamera.width * oxfCamera.height != distortionLUT_center_x.cols)
			throw runtime_error("Mismatched image size and model size");
		distortionLUT_center_x = distortionLUT_center_x.reshape(0, oxfCamera.height);
		distortionLUT_center_y = distortionLUT_center_y.reshape(0, oxfCamera.height);
	}

	cv::Mat target;
	cv::remap(src, target, distortionLUT_center_x, distortionLUT_center_y, cv::INTER_LINEAR);
	return target;
}
