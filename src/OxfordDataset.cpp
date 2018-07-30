/*
 * OxfordDataset.cpp
 *
 *  Created on: Jul 28, 2018
 *      Author: sujiwo
 */


#include <cstdio>
#include <Eigen/Eigen>

#include "csv.h"
#include "OxfordDataset.h"


using namespace std;

static const set<int> GpsColumns ({0,9,8,4,2,3});


OxfordDataset::OxfordDataset(const std::string &dirpath, GroundTruthSrc gts) :
	oxPath (dirpath)
{
	loadTimestamps();
	loadGps();
	loadIns();
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
	gpsTimestamps.resize(ss);

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


void
OxfordDataset::loadIns()
{
	const string insFilePath = oxPath + "/gps/ins.csv";
}


void OxfordDataset::loadTimestamps()
{
	const string timestampsPath = oxPath + "/stereo.timestamps";
	StringTable TS = create_table(timestampsPath);
	const size_t ss = TS.size();
	stereoTimestamps.resize(ss);
	stereoImagePaths.resize(ss);

	for (uint32_t i=0; i<ss; i++) {
		stereoTimestamps[i] = stoul(TS.get(i,0));

		const string
			ctrname = oxPath + "/stereo/centre",
        	lftname = oxPath + "/stereo/left",
			rhtname = oxPath + "/stereo/right";

		stereoImagePaths[i][StereoImagePath::LEFT] = lftname + '/' + TS.get(i,0) + ".png";
		stereoImagePaths[i][StereoImagePath::CENTER] = ctrname + '/' + TS.get(i,0) + ".png";
		stereoImagePaths[i][StereoImagePath::RIGHT] = rhtname + '/' + TS.get(i,0) + ".png";
	}
}
