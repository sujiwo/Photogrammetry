/*
 * Frame.h
 *
 *  Created on: Jul 17, 2018
 *      Author: sujiwo
 */

#ifndef FRAME_H_
#define FRAME_H_


#include <opencv2/core.hpp>
#include <vector>


class Frame
{
public:
	Frame();
	virtual ~Frame();

protected:
	cv::Mat image;
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat descriptors;
};

#endif /* FRAME_H_ */
