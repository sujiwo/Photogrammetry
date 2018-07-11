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
#include "DBoW2/FORB.h"
#include "DBoW2/TemplatedVocabulary.h"
#include "VMap.h"



typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>
	ORBVocabulary;


class ImageDatabase
{

public:
	ImageDatabase (VMap *_m);
	virtual ~ImageDatabase();

	void rebuild ();

	// ???
	const kfid find ();

private:
	ORBVocabulary myVoc;
	std::vector<std::set<kfid> > invertedKeywordDb;
	VMap *cMap;
};

#endif /* IMAGEDATABASE_H_ */
