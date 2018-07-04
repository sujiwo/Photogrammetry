/*
 * KeyFrame.cpp
 *
 *  Created on: Jun 5, 2018
 *      Author: sujiwo
 */

#include "KeyFrame.h"
#include <exception>
#include "MapBuilder.h"

using namespace std;
using namespace Eigen;


kfid KeyFrame::nextId = 0;


Vector2d cv2eigen (const cv::Point2f &p)
{ return Eigen::Vector2d (p.x, p.y); }

typedef Matrix<double,3,4> poseMatrix;
typedef Matrix4d poseMatrix4;


#define pixelReprojectionError 6.0


KeyFrame::KeyFrame(
		const cv::Mat &imgSrc,
	const Vector3d &p, const Eigen::Quaterniond &o,
	cv::Mat &mask,
	cv::Ptr<cv::FeatureDetector> fdetector,
	const CameraPinholeParams *cameraIntr) :

	orientation(o),
	position(p),
//	prev(NULL),
	id (nextId++)
{
	normal = externalParamMatrix().block(0,0,3,3).transpose().col(2);

	fdetector->detectAndCompute(imgSrc, mask, keypoints, descriptors, false);

	Matrix<double,3,4> camInt = cameraIntr->toMatrix();
	projMatrix = cameraIntr->toMatrix() * externalParamMatrix4();
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


poseMatrix4 KeyFrame::externalParamMatrix4() const
{
	poseMatrix4 ex = poseMatrix4::Identity();
	Matrix3d R = orientation.toRotationMatrix().transpose();
	ex.block<3,3>(0,0) = R;
	ex.col(3).head(3) = -(R*position);
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
			FeaturePair fp = {m.trainIdx, k1.keypoints[m.trainIdx].pt, m.queryIdx, k2.keypoints[m.queryIdx].pt};
			featurePairs.push_back (fp);
		}
	}
}


void KeyFrame::triangulate (
	const KeyFrame *kf1, const KeyFrame *kf2,
	std::vector<mpid> &mapPointList,
	const std::vector<FeaturePair> &featurePairs,
	std::map<mpid, kpid> &mapPointToKeyPointInKeyFrame1,
	std::map<mpid, kpid> &mapPointToKeyPointInKeyFrame2,
	VMap *parent
)
{
	set<uint> badMatches;

	const poseMatrix &pm1 = kf1->projMatrix,
		&pm2 = kf2->projMatrix;

	mapPointList.clear();

	for (uint i=0; i<featurePairs.size(); i++) {
		auto &fp = featurePairs[i];
		Vector2d proj1 = cv2eigen(fp.keypoint1),
			proj2 = cv2eigen(fp.keypoint2);

		Vector4d triangulatedpt;
		TriangulateDLT (pm1, pm2, proj1, proj2, triangulatedpt);
		Vector3d pointm = triangulatedpt.head(3);

		// Check for Reprojection Errors
		float pj1 = (kf1->project(pointm) - proj1).norm(),
			pj2 = (kf2->project(pointm) - proj2).norm();
		if (pj1 > pixelReprojectionError or pj2 > pixelReprojectionError)
			continue;

		// checking for regularity of triangulation result
		// 1: Point must be in front of camera
		Vector3d v1 = pointm - kf1->position;
		double cos1 = v1.dot(kf1->normal) / v1.norm();
		if (cos1 < 0)
			continue;
		double dist1 = v1.norm();
		Vector3d v2 = pointm - kf2->position;
		double cos2 = v2.dot(kf2->normal) / v2.norm();
		if (cos2 < 0)
			continue;
		double dist2 = v2.norm();

		// 2: Must have enough parallax (ie. remove faraway points)
		double cosParallax = (-v1).dot(-v2) / (dist1 * dist2);
		if (cosParallax >= 0.999990481)
			continue;

		mpid newMp = parent->createMapPoint(pointm);
		mapPointList.push_back(newMp);
		mapPointToKeyPointInKeyFrame1[newMp] = fp.kpid1;
		mapPointToKeyPointInKeyFrame2[newMp] = fp.kpid2;
	}
}



Vector2d KeyFrame::project(const Vector3d &pt3) const
{
	Vector3d ptx = projMatrix * pt3.homogeneous();
	return ptx.head(2) / ptx[2];
}
