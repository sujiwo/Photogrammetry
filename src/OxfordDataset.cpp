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
#include "utilities.h"


using namespace std;
using namespace Eigen;


static const set<int> GpsColumns ({0,9,8,4,2,3});
static const set<int> InsColumns ({0,6,5,4,12,13,14,10,9,11,2,3});


OxfordDataset::OxfordDataset(const std::string &dirpath, GroundTruthSrc gts) :
	oxPath (dirpath)
{
	loadTimestamps();
	loadGps();
	loadIns();

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
			is.pitch = stod(INS_s.get(i, "pitch"));
			is.yaw = stod(INS_s.get(i, "yaw"));
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


TTransform fromINS(const InsPose &ps)
{
	TTransform px;
	px.position = Vector3d(ps.easting, ps.northing, ps.altitude);
	px.position.x() += OriginCorrectionEasting;
	px.position.y() += OriginCorrectionNorthing;
	px.orientation = fromRPY(ps.roll, ps.pitch, ps.yaw);
	return px;
}


TTransform interpolateFromINS (
	uint64_t timestamp,
	const InsPose &ps1,
	const InsPose &ps2)
{
	assert(timestamp >= ps1.timestamp and timestamp<=ps2.timestamp);
	TTransform px;

	TTransform px1 = fromINS(ps1),
		px2 = fromINS(ps2);

	double ratio = double(timestamp - ps1.timestamp) / double(ps2.timestamp - ps1.timestamp);
	px.position = px1.position + ratio * (px2.position - px1.position);
	px.orientation = px1.orientation.slerp(ratio, px2.orientation);

	return px;
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
		TTransform px;

		// edge case min
		if (ts < insPoseTable[0].timestamp) {
			px = fromINS(insPoseTable[0]);
		}

		else if (ts > insPoseTable[insPoseTable.size()-1].timestamp) {
			px = fromINS(insPoseTable[insPoseTable.size()-1]);
		}

		else {
			decltype(Itx) Itx_prev;
//			while(ts < (*Itx).first) {
//				Itx_prev = Itx;
//				Itx++;
//			}
			do {
				Itx_prev = Itx;
				Itx++;
			} while (ts < (*Itx).first);

			px = interpolateFromINS(ts,
				*((*Itx_prev).second),
				*((*Itx).second));
		}

		stereoGroundTruths.insert(make_pair(ts, px));
	}
}
