/*
 * optimizer.cpp
 *
 *  Created on: Jun 26, 2018
 *      Author: sujiwo
 */


#include "optimizer.h"
#include "KeyFrame.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/linear_solver_eigen.h"
#include "g2o/types/types_six_dof_expmap.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/solvers/linear_solver_dense.h"
#include "g2o/types/types_seven_dof_expmap.h"


using namespace std;
using namespace Eigen;


typedef uint64 oid;

const float thHuber2D = sqrt(5.99);
const float thHuber3D = sqrt(7.815);


// XXX: Wrong
g2o::SE3Quat toSE3Quat
(const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation)
{ return g2o::SE3Quat(orientation, position); }


g2o::SE3Quat toSE3Quat (const KeyFrame &kf)
{
	Matrix4d extMat = kf.externalParamMatrix4();
	return g2o::SE3Quat(extMat.block<3,3>(0,0), extMat.block<3,1>(0,3));
}


// XXX: Wrong
void fromSE3Quat(const g2o::SE3Quat &pose,
	Vector3d &position, Quaterniond &orientation)
{
	position = pose.translation();
	orientation = pose.rotation();
}


void fromSE3Quat (const g2o::SE3Quat &pose, KeyFrame &kf)
{
	kf.getOrientation() = pose.rotation().inverse();
	kf.getPosition() = -(kf.getOrientation() * pose.translation());
}


#include <cstdio>
void debugVector (const VectorXd &v)
{
	if (v.size()==2)
		printf ("(%f, %f)\n", v.x(), v.y());
	else if (v.size()==3)
		printf ("(%f, %f, %f)\n", v.x(), v.y(), v.z());
}


void bundle_adjustment (VMap *orgMap)
{
	vector<kfid> keyframeList = orgMap->allKeyFrames();
	vector<mpid> mappointList = orgMap->allMapPoints();

    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(true);
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;
    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();
    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

	// XXX: This routine does not support multiple camera
	const CameraPinholeParams camPtr = orgMap->getCameraParameter(0);
	g2o::CameraParameters *camParams =
		new g2o::CameraParameters(camPtr.fx, Vector2d(camPtr.cx,camPtr.cy), 0);
	camParams->setId(0);
	optimizer.addParameter(camParams);

	map<oid, kfid> vertexKfMap;
	map<kfid, g2o::VertexSE3Expmap*> vertexKfMapInv;
	map<oid, mpid> vertexMpMap;
	map<mpid, g2o::VertexSBAPointXYZ*> vertexMpMapInv;
	oid vId = 0;

	for (kfid &kId: keyframeList) {

		g2o::VertexSE3Expmap *vKf = new g2o::VertexSE3Expmap();
		KeyFrame *kf = orgMap->keyframe(kId);
		vKf->setEstimate (toSE3Quat(*kf));
		vKf->setId(vId);
		vKf->setFixed(kId<2);
		optimizer.addVertex(vKf);
		vertexKfMap.insert(pair<oid, kfid> (vId, kId));
		vertexKfMapInv[kId] = vKf;
		vId ++;

		for (auto &ptr: orgMap->allMapPointsAtKeyFrame(kId)) {

			const MapPoint *mp = orgMap->mappoint(ptr.first);
			const cv::KeyPoint p2K = kf->getKeyPointAt(ptr.second);

			g2o::VertexSBAPointXYZ *vMp = new g2o::VertexSBAPointXYZ();
			vMp->setEstimate(mp->getPosition());
			vMp->setMarginalized(true);
			vMp->setId(vId);
			optimizer.addVertex(vMp);
			vertexMpMap.insert(pair<oid,mpid> (vId, mp->getId()));
			vertexMpMapInv[mp->getId()] = vMp;
			vId++;

			// Edges
			g2o::EdgeSE3ProjectXYZ *edge = new g2o::EdgeSE3ProjectXYZ();
			edge->setVertex(0, vMp);
			edge->setVertex(1, vKf);
			edge->setMeasurement(Vector2d(p2K.pt.x, p2K.pt.y));
			edge->fx = camPtr.fx;
			edge->fy = camPtr.fy;
			edge->cx = camPtr.cx;
			edge->cy = camPtr.cy;

			Matrix2d uncertainty = Matrix2d::Identity() * (1.2*(p2K.octave+1));
			edge->setInformation(uncertainty);
			optimizer.addEdge(edge);
		}
	}

	optimizer.initializeOptimization();
	// XXX: Determine number of iterations
	optimizer.optimize(10);

	// Recovery of optimized data
	// KeyFrames
	for (auto &kVpt: vertexKfMap) {

		oid vId = kVpt.first;
		kfid kId = kVpt.second;
		KeyFrame *kf = orgMap->keyframe(kId);
		g2o::VertexSE3Expmap *vKfSE3 = static_cast<g2o::VertexSE3Expmap*> (optimizer.vertex(vId));

		g2o::SE3Quat kfPoseSE3 = vKfSE3->estimate();
		fromSE3Quat(kfPoseSE3, *kf);
	}

	// MapPoints
	for (auto &mVpt: vertexMpMap) {
		oid vId = mVpt.first;
		mpid mid = mVpt.second;
		MapPoint *mp = orgMap->mappoint(mid);
		g2o::VertexSBAPointXYZ *vMp = static_cast<g2o::VertexSBAPointXYZ*> (optimizer.vertex(vId));
		mp->setPosition(vMp->estimate());
	}
}
