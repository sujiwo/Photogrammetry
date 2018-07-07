/*
 * Map.cpp
 *
 *  Created on: Jun 13, 2018
 *      Author: sujiwo
 */

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

#include "VMap.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "MapObjectSerialization.h"
#include "utilities.h"



using namespace Eigen;
using namespace std;


VMap::VMap()
{ }


VMap::VMap(
	const cv::Mat &m,
	cv::Ptr<cv::FeatureDetector> fdetect,
	cv::Ptr<cv::DescriptorMatcher> dmatcher) :
		featureDetector(fdetect),
		descriptorMatcher(dmatcher)
{
	mask = m.clone();
}


VMap::~VMap()
{}


Matrix<double,3,4> CameraPinholeParams::toMatrix() const
{
	Matrix<double,3,4> K = Matrix<double,3,4>::Zero();
	K(2,2) = 1.0;
    K(0,0) = fx;
    K(1,1) = fy;
    K(0,2) = cx;
    K(1,2) = cy;
	return K;
}


kfid VMap::createKeyFrame(const cv::Mat &imgSrc,
		const Eigen::Vector3d &p, const Eigen::Quaterniond &o,
		KeyFrame **ptr)
{
	KeyFrame *nKf = new KeyFrame(imgSrc, p, o, mask, featureDetector, &camera);
	kfid nId = nKf->getId();
	keyframeInvIdx.insert(pair<kfid,KeyFrame*> (nId, nKf));

	if (ptr!=NULL)
		*ptr = nKf;
	return nId;
}


mpid VMap::createMapPoint(const Vector3d &p, MapPoint **ptr)
{
	MapPoint *mp = new MapPoint(p);
	mpid nId = mp->getId();
	mappointInvIdx.insert(pair<mpid,MapPoint*> (nId, mp));

	if (ptr!=NULL)
		*ptr = mp;
	return nId;
}


KeyFrame* VMap::getKeyFrameById (const kfid &i) const
{
	return keyframeInvIdx.at(i);
}


void VMap::estimateStructure(const kfid &kfid1, const kfid &kfid2)
{
	const KeyFrame
		*kf1 = getKeyFrameById(kfid1),
		*kf2 = getKeyFrameById(kfid2);

	vector<FeaturePair> featurePairs_1_2;
	KeyFrame::match(*kf1, *kf2, descriptorMatcher, featurePairs_1_2);

	std::map<mpid, kpid> &kf1kp = framePoints[kfid1];
	std::map<mpid, kpid> &kf2kp = framePoints[kfid2];

	vector<mpid> newMapPointList;
	KeyFrame::triangulate(kf1, kf2, newMapPointList, featurePairs_1_2,
		framePoints[kfid1], framePoints[kfid2],
		this);

	// Update visibility information
	for (mpid &mpidx: newMapPointList) {
		pointAppearances[mpidx].insert(kfid1);
		pointAppearances[mpidx].insert(kfid2);
	}
}


void
VMap::estimateAndTrack (const kfid &kfid1, const kfid &kfid2)
{
	const KeyFrame
		*kf1 = getKeyFrameById(kfid1),
		*kf2 = getKeyFrameById(kfid2);

	// Track Map Points from KF1 that are visible in KF2
	map<kpid,mpid> kf1kp2mp = reverseMap(framePoints[kfid1]);
	set<kpid> kp1s = allKeys(kf1kp2mp);
	vector<FeaturePair> pairList12;
	KeyFrame::matchSubset(*kf1, *kf2, descriptorMatcher, pairList12, kp1s);
	if (pairList12.size() > 0) {
		for (auto &p: pairList12) {
			mpid ptId = kf1kp2mp[p.kpid1];
			// This particular mappoint is visible in KF2
			pointAppearances[ptId].insert(kfid2);
			framePoints[kfid2][ptId] = p.kpid2;
		}
	}

	// Estimate new mappoints that are visible in KF1 & KF2
	set<kpid> kp1rest = subtract(KeyFrame::allKeyPointId(*kf1), kp1s);
	set<kpid> kp2s = allKeys(reverseMap(framePoints[kfid2]));
	set<kpid> kp2rest = subtract(KeyFrame::allKeyPointId(*kf2), kp2s);
	pairList12.clear();
	KeyFrame::matchSubset(*kf1, *kf2, descriptorMatcher, pairList12, kp1rest, kp2rest);
	vector<mpid> newMapPointList;
	KeyFrame::triangulate(kf1, kf2, newMapPointList, pairList12,
		framePoints[kfid1], framePoints[kfid2],
		this);

	// Update visibility information
	for (mpid &mpidx: newMapPointList) {
		pointAppearances[mpidx].insert(kfid1);
		pointAppearances[mpidx].insert(kfid2);
	}
}


vector<kfid>
VMap::allKeyFrames () const
{
	vector<kfid> kfIdList(keyframeInvIdx.size());
	for (auto &key: keyframeInvIdx) {
		kfIdList.push_back(key.first);
	}
	return kfIdList;
}


vector<mpid>
VMap::allMapPoints () const
{
	vector<mpid> mpIdList(mappointInvIdx.size());
	for (auto &key: mappointInvIdx) {
		mpIdList.push_back(key.first);
	}
	return mpIdList;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr
VMap::dumpPointCloudFromMapPoints ()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr mapPtCl
		(new pcl::PointCloud<pcl::PointXYZ>(mappointInvIdx.size(), 1));

	uint64 i = 0;
	for (auto &mpIdx: mappointInvIdx) {
		MapPoint *mp = mpIdx.second;
		mapPtCl->at(i).x = mp->X();
		mapPtCl->at(i).y = mp->Y();
		mapPtCl->at(i).z = mp->Z();
		i++;
	}

	return mapPtCl;
}


struct MapFileHeader {
	uint64
		numOfKeyFrame,
		numOfMapPoint;
};


bool
VMap::save(const string &filepath)
{
	MapFileHeader header;
	header.numOfKeyFrame = keyframeInvIdx.size();
	header.numOfMapPoint = mappointInvIdx.size();

	fstream mapFileFd;
	mapFileFd.open (filepath, fstream::out | fstream::trunc);
	if (!mapFileFd.is_open())
		throw runtime_error("Unable to create map file");
	mapFileFd.write((const char*)&header, sizeof(header));

	boost::archive::binary_oarchive mapStore (mapFileFd);

	for (auto &kfptr : keyframeInvIdx) {
		KeyFrame *kf = kfptr.second;
		mapStore << *kf;
	}

	for (auto &mpPtr : mappointInvIdx) {
		MapPoint *mp = mpPtr.second;
		mapStore << *mp;
	}

	mapStore << pointAppearances;
	mapStore << framePoints;
	mapStore << mask;

	return true;
}


bool
VMap::load(const string &filepath)
{
	MapFileHeader header;

	fstream mapFileFd;
	mapFileFd.open(filepath, fstream::in);
	if (!mapFileFd.is_open())
		throw runtime_error(string("Unable to open map file: ") + filepath);
	mapFileFd.read((char*)&header, sizeof(header));

	boost::archive::binary_iarchive mapStore (mapFileFd);

	KeyFrame *kfArray = new KeyFrame[header.numOfKeyFrame];
	MapPoint *mpArray = new MapPoint[header.numOfMapPoint];

	for (int i=0; i<header.numOfKeyFrame; i++) {
		mapStore >> kfArray[i];
	}
	for (int j=0; j<header.numOfMapPoint; j++) {
		mapStore >> mpArray[j];
	}

	mapStore >> pointAppearances;
	mapStore >> framePoints;
	mapStore >> mask;

	// Rebuild pointers
	keyframeInvIdx.clear();
	for (int i=0; i<header.numOfKeyFrame; i++) {
		KeyFrame *kf = &(kfArray[i]);
		keyframeInvIdx.insert(pair<kfid,KeyFrame*>(kf->getId(), kf));
	}

	mappointInvIdx.clear();
	for (int j=0; j<header.numOfMapPoint; j++) {
		MapPoint *mp = &(mpArray[j]);
		mappointInvIdx.insert(pair<mpid,MapPoint*>(mp->getId(), mp));
	}

	return true;
}


vector<pair<Vector3d,Quaterniond> >
VMap::dumpCameraPoses () const
{
	vector<pair<Vector3d,Quaterniond> > Retr;

	for (auto &kptr: keyframeInvIdx) {
		KeyFrame *kf = kptr.second;
		Retr.push_back(
			pair<Vector3d,Quaterniond>
				(kf->getPosition(), kf->getOrientation()));
	}

	return Retr;
}


vector<kfid>
VMap::getKeyFrameList() const
{
	vector<kfid> ret;
	for (auto &kf: keyframeInvIdx)
		ret.push_back(kf.first);
	return ret;
}


std::vector<mpid>
VMap::getMapPointList() const
{
	vector<mpid> ret;
	for (auto &mp: mappointInvIdx)
		ret.push_back(mp.first);
	return ret;
}
