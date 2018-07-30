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


#include "VMap.h"
#include "utilities.h"


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


struct TTransform
{
	Eigen::Vector3d position;
	Eigen::Quaterniond orientation;
};


class OxfordDataset
{
public:
	OxfordDataset (const std::string &dirpath, GroundTruthSrc gts=GroundTruthSrc::INS);
	virtual ~OxfordDataset();

	uint size() const;

	CameraPinholeParams getCameraParameter() const
	{ return oxfCamera; }

protected:
	std::vector<DataItem> records;
	std::vector<unsigned long int> timestamps;
	CameraPinholeParams oxfCamera;

	std::string oxPath;

	std::vector<uint64_t> stereoTimestamps;
	std::vector<StereoImagePath> stereoImagePaths;
	std::map<uint64_t,TTransform> stereoGroundTruths;

	std::vector<GpsPose> gpsPoseTable;
//	std::vector<uint64_t> gpsTimestamps;

	std::vector<InsPose> insPoseTable;
//	std::vector<uint64_t> insTimestamps;

private:
	void loadIns ();
	void loadGps ();
	void loadTimestamps ();

	void createStereoGroundTruths();
};

#endif /* _OXFORDDATASET_H_ */
