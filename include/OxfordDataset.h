/*
 * OxfordDataset.h
 *
 *  Created on: Jul 28, 2018
 *      Author: sujiwo
 */

#ifndef _OXFORDDATASET_H_
#define _OXFORDDATASET_H_

#include <string>
#include <iostream>
#include <vector>
#include <tuple>
#include <array>
#include <set>
#include <opencv2/opencv.hpp>

#include "VMap.h"
#include "utilities.h"
#include "GenericDataset.h"



/*
 * XXX: Oxford Timestamp is in Microsecond
 */


enum GroundTruthSrc {
	GPS,
	INS
};


/*
 * These values are constants to shift X/Y coordinates to more `reasonable' values
 */
const double
	OriginCorrectionEasting = -620248.53,
	OriginCorrectionNorthing = -5734882.47;

struct StereoImagePath: public std::array <std::string,3>
{
public:
	enum {
		LEFT=0,
		CENTER=1,
		RIGHT=2
	};
};


struct GpsPose
{
	uint64_t timestamp;
	double
		easting,
		northing,
		altitude,
		latitude,
		longitude;
};


struct InsPose : public GpsPose
{
	double
		roll,
		pitch,
		yaw;
};


//struct TTransform
//{
//	Eigen::Vector3d position;
//	Eigen::Quaterniond orientation;
//
//	Transform3d toEig();
//};


class OxfordDataset;
struct OxfordDataItem : public GenericDataItem
{
	uint64_t timestamp;
	StereoImagePath paths;
	Pose groundTruth;
	OxfordDataset *parent;

	cv::Mat getImage (int which=StereoImagePath::CENTER);
	cv::Mat getImage () const;
	Eigen::Vector3d getPosition() const;
	Eigen::Quaterniond getOrientation() const;

};


class OxfordDataset
{
public:
	OxfordDataset (const std::string &dirpath, const std::string &modelDir, GroundTruthSrc gts=GroundTruthSrc::INS);
	virtual ~OxfordDataset();

	uint size() const;

//	CameraPinholeParams getCameraParameter(const std::string &yamlFileCameraParams=std::string());

	void dumpGroundTruth(const std::string &fp=std::string());

	OxfordDataItem at(const int i);

	friend struct OxfordDataItem;
	cv::Mat undistort (cv::Mat &src);


protected:
	std::vector<DataItem> records;
	std::vector<unsigned long int> timestamps;
	CameraPinholeParams oxfCamera;

	std::string oxPath;

	std::vector<uint64_t> stereoTimestamps;
	std::vector<StereoImagePath> stereoImagePaths;
	std::map<uint64_t,Pose> stereoGroundTruths;

	std::vector<GpsPose> gpsPoseTable;
//	std::vector<uint64_t> gpsTimestamps;

	std::vector<InsPose> insPoseTable;
//	std::vector<uint64_t> insTimestamps;

	cv::Mat distortionLUT_center_x, distortionLUT_center_y;

private:
	void loadIns ();
	void loadGps ();
	void loadTimestamps ();

	void createStereoGroundTruths();

	void loadModel (const std::string &modelDir);
};

#endif /* _OXFORDDATASET_H_ */
