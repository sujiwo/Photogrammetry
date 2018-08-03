/*
 * GenericDataset.h
 *
 *  Created on: Aug 3, 2018
 *      Author: sujiwo
 */

#ifndef _GENERICDATASET_H_
#define _GENERICDATASET_H_


#include <sys/types.h>
#include <string>
#include <Eigen/Eigen>
#include <opencv2/core.hpp>
#include "VMap.h"


typedef uint64_t dataItemId;


class GenericDataset;


class GenericDataItem
{
public:
	virtual ~GenericDataItem();
	virtual cv::Mat getImage() const = 0;
	virtual Eigen::Vector3d getPosition() const = 0;
	virtual Eigen::Quaterniond getOrientation() const = 0;
};


class GenericDataset
{
public:

	virtual ~GenericDataset();

	virtual uint size() const = 0;

	virtual CameraPinholeParams getCameraParameter() = 0;

	virtual cv::Mat getMask() = 0;

	const GenericDataItem& at(const int i);
};




#endif /* _GENERICDATASET_H_ */
