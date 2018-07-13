/*
 * ImageDatabase.h
 *
 *  Created on: Jul 11, 2018
 *      Author: sujiwo
 */

#ifndef IMAGEDATABASE_H_
#define IMAGEDATABASE_H_


#include <vector>
#include <set>
#include <map>
#include <boost/serialization/serialization.hpp>
#include "DBoW2/BowVector.h"
#include "DBoW2/FORB.h"
#include "DBoW2/TemplatedVocabulary.h"
#include "VMap.h"
#include "cvobj_serialization.h"



typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>
	ORBVocabulary;


class ImageDatabase
{

public:
	ImageDatabase (VMap *_m);
	virtual ~ImageDatabase();

	void addKeyFrame (const kfid &kfId);

	void rebuildAll ();

	// Used for loop detection
	const kfid find (const KeyFrame *kf);

protected:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive &ar, const unsigned int file_version)
	{
		ar & invertedKeywordDb;
		ar & BoWList;
//		ar & FeatVecList;
	}

private:
	ORBVocabulary myVoc;
	std::map<DBoW2::WordId, std::set<kfid> > invertedKeywordDb;

	VMap *cMap;

	std::map<kfid, DBoW2::BowVector> BoWList;
	std::map<kfid, DBoW2::FeatureVector> FeatVecList;
};

#endif /* IMAGEDATABASE_H_ */
