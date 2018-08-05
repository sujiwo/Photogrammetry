/*
 * CustomDataset.h
 *
 *  Created on: Aug 4, 2018
 *      Author: sujiwo
 */

#ifndef CUSTOMDATASET_H_
#define CUSTOMDATASET_H_

#include <string>
#include <vector>
#include "GenericDataset.h"
#include "VMap.h"


class CustomDataset;
class CustomDataItem : public GenericDataItem
{
	friend class CustomDataset;
	CustomDataItem() {}

	cv::Mat getImage() const;

	Eigen::Vector3d getPosition() const
	{ return position; }

	Eigen::Quaterniond getOrientation() const
	{ return orientation; }

	uint64_t getId() const
	{ return id; }

protected:
	uint64_t id;
	std::string imagePath;
	Eigen::Vector3d position;
	Eigen::Quaterniond orientation;
};


class CustomDataset : public GenericDataset
{
public:

	CustomDataset(const std::string &dataDirPath);
	virtual ~CustomDataset();

	size_t size() const;

	CameraPinholeParams getCameraParameter();

	cv::Mat getMask()
	{ return mask; }

	const GenericDataItem& at(const int i);

protected:
	const std::string rootPath;

	std::vector<CustomDataItem> records;

	CameraPinholeParams cparams;

	cv::Mat mask;
};

#endif /* CUSTOMDATASET_H_ */
