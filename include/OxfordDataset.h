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


#include "VMap.h"
#include "utilities.h"


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

private:
	void loadIns ();
	void loadGps ();
	void loadTimestamps ();
};

#endif /* INCLUDE_OXFORDDATASET_H_ */
