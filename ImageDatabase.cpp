/*
 * ImageDatabase.cpp
 *
 *  Created on: Jul 11, 2018
 *      Author: sujiwo
 */

#include <map>
#include "ImageDatabase.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "Frame.h"


using namespace std;
using namespace Eigen;


ImageDatabase::ImageDatabase(VMap *_m) :
	cMap(_m)
{
	// TODO Auto-generated constructor stub

}


ImageDatabase::~ImageDatabase()
{
	// TODO Auto-generated destructor stub
}


std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors)
{
    std::vector<cv::Mat> vDesc;
    vDesc.reserve(Descriptors.rows);
    for (int j=0;j<Descriptors.rows;j++)
        vDesc.push_back(Descriptors.row(j));

    return vDesc;
}


void
ImageDatabase::rebuildAll()
{
	// 1: Build Map Points' Descriptors
//#pragma omp parallel

	for (const mpid &mid: cMap->getMapPointList()) {

		MapPoint *mp = cMap->mappoint(mid);
		vector<KeyMapPoint> kfkp;

		for (auto &kid: cMap->getRelatedKeyFrames(mid)) {
			KeyMapPoint kmp = {
				cMap->keyframe(kid),
				cMap->getKeyPointId(kid, mid)};
			kfkp.push_back(kmp);
		}

		mp->createDescriptor(kfkp);
	}

	// 2: Rebuild Vocabulary
	vector<vector<DBoW2::FORB::TDescriptor> > keymapFeatures;
	keymapFeatures.reserve(cMap->numOfKeyFrames());
//#pragma omp parallel
	for (auto &kid: cMap->allKeyFrames()) {
		vector<cv::Mat> kfDescriptor;

		map<mpid,kpid> mappts = cMap->allMapPointsAtKeyFrame(kid);
		for (auto &mpptr: mappts) {
			cv::Mat mpDescriptor = cMap->mappoint(mpptr.first)->getDescriptor();
			kfDescriptor.push_back(mpDescriptor);
		}

		keymapFeatures.push_back(kfDescriptor);
	}

	myVoc.create(keymapFeatures);

	// 3: Compute BoW & Feature Vectors
//#pragma omp parallel
	for (auto &kid: cMap->allKeyFrames()) {
		KeyFrame *kf = cMap->keyframe(kid);
		vector<cv::Mat> kfDescs = toDescriptorVector(kf->getDescriptors());

		BoWList[kid] = DBoW2::BowVector();
		FeatVecList[kid] = DBoW2::FeatureVector();
		myVoc.transform(kfDescs, BoWList[kid], FeatVecList[kid], 4);

		// Build Inverse Index
		for (auto &bowvec: BoWList[kid]) {
			const DBoW2::WordId wrd = bowvec.first;
			invertedKeywordDb[wrd].insert(kid);
		}
	}
}


kfid
ImageDatabase::find (const KeyFrame *kf) const
{
	return 0;
}


kfid
ImageDatabase::find (Frame &f, bool simple) const
{
	f.computeBoW(*this);

	map<kfid, uint> kfRelated;
	for (auto &bWrdPtr : f.getWords()) {
		auto wid = bWrdPtr.first;

		try {
			const set<kfid> &relatedKf = invertedKeywordDb.at(wid);
			for (const kfid &i: relatedKf) {
				try {
					kfRelated.at(i) += 1;
				} catch (out_of_range&) {
					kfRelated.at(i) = 1;
				}
			}
		} catch (out_of_range&) {
			continue;
		}
	}

	if (simple) {
		uint wCnt=0;
		kfid mx;
		for (auto &bPtr: kfRelated) {
			if (bPtr.second > wCnt) {
				mx = bPtr.first;
				wCnt = bPtr.second;
			}
		}
		return mx;
	}
}
