/*
 * KeyFrame.cpp
 *
 *  Created on: Jun 5, 2018
 *      Author: sujiwo
 */

#include "KeyFrame.h"
#include <exception>

using namespace std;
using namespace Eigen;


int64 KeyFrame::nextId = 0;

Vector2d cv2eigen (const cv::Point2f &p)
{ return Eigen::Vector2d (p.x, p.y); }

typedef Matrix<double,3,4> poseMatrix;


KeyFrame::KeyFrame(
	const string &path,
	const Vector3d &p, const Eigen::Quaterniond &o,
	cv::Mat &mask,
	cv::Ptr<cv::FeatureDetector> fdetector ) :

	orientation(o),
	position(p),
	prev(NULL),
	id (nextId++)
{
	normal = externalParamMatrix().block(0,0,3,3).transpose().col(2);
	image = cv::imread(path, cv::IMREAD_GRAYSCALE);
	if (image.empty()==true)
		throw runtime_error("Unable to open image file");

	fdetector->detectAndCompute(image, mask, keypoints, descriptors, false);
}


KeyFrame::~KeyFrame()
{
	// TODO Auto-generated destructor stub
}


// XXX: 3x4 or 4x4 ?
poseMatrix KeyFrame::externalParamMatrix () const
{
	poseMatrix ex = poseMatrix::Zero();
	Matrix3d R = orientation.toRotationMatrix().transpose();
	ex.block<3,3>(0,0) = R;
	ex.col(3) = -(R*position);
	return ex;
}


void KeyFrame::match(const KeyFrame &k1, const KeyFrame &k2,
	cv::Ptr<cv::DescriptorMatcher> matcher,
	vector<FeaturePair> &featurePairs
)
{
	vector<cv::DMatch> k12matches;
	matcher->match(k1.descriptors, k2.descriptors, k12matches);

	for (auto &m: k12matches) {
		if (m.trainIdx < k1.keypoints.size() and m.queryIdx < k2.keypoints.size()) {
			FeaturePair fp = {k1.id, k1.keypoints[m.trainIdx].pt, k2.id, k2.keypoints[m.queryIdx].pt};
			featurePairs.push_back (fp);
		}
	}
}


void KeyFrame::triangulate (
	const KeyFrame &kf1, const KeyFrame &kf2,
	vector<MapPoint*> ptsList,
	const vector<FeaturePair> &featurePairs )
{
	set<uint> badMatches;

	poseMatrix pm1 = kf1.externalParamMatrix(),
		pm2 = kf2.externalParamMatrix();

	for (uint i=0; i<featurePairs.size(); i++) {

		auto &fp = featurePairs[i];
		Vector2d proj1 = cv2eigen(fp.keypoint1),
			proj2 = cv2eigen(fp.keypoint2);

		Vector4d triangulatedpt;
		Triangulate (pm1, pm2, proj1, proj2, &triangulatedpt);
		Vector3d pointm = triangulatedpt.head(3) / triangulatedpt[3];

		// checking for regularity of triangulation result
		// 1: Point must be in front of camera
		Vector3d v1 = pointm - kf1.position;
		double cos1 = v1.dot(kf1.normal) / v1.norm();
		if (cos1 < 0)
			continue;
		Vector3d v2 = pointm - kf2.position;
		double cos2 = v2.dot(kf2.normal) / v2.norm();
		if (cos2 < 0)
			continue;

		// 2: Must have enough parallax (ie. remove faraway points)


//		MapPoint *npoint = new MapPoint;
//		ptsList.push_back(npoint);
	}
}
