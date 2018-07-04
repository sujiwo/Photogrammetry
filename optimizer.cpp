/*
 * optimizer.cpp
 *
 *  Created on: Jun 26, 2018
 *      Author: sujiwo
 */


#include "optimizer.h"
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


g2o::SE3Quat toSE3Quat
(const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation)
{ return g2o::SE3Quat(orientation, position); }


void fromSE3Quat(const g2o::SE3Quat &pose,
	Vector3d &position, Quaterniond &orientation)
{
	position = pose.translation();
	orientation = pose.rotation();
}


void bundle_adjustment (VMap *orgMap)
{
	vector<kfid> keyframeList = orgMap->allKeyFrames();
	vector<mpid> mappointList = orgMap->allMapPoints();

    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;
    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

	map<oid, kfid> vertexKfMap;
	map<kfid, g2o::VertexSE3Expmap*> vertexKfMapInv;
	map<oid, mpid> vertexMpMap;
	map<mpid, g2o::VertexSBAPointXYZ*> vertexMpMapInv;
	oid vId = 0;

	for (kfid &kId: keyframeList) {

		g2o::VertexSE3Expmap *vKf = new g2o::VertexSE3Expmap();
		KeyFrame *kf = orgMap->keyframe(kId);
		vKf->setEstimate (toSE3Quat(kf->getPosition(), kf->getOrientation()));
		vKf->setId(vId);
		optimizer.addVertex(vKf);
		vertexKfMap.insert(pair<oid, kfid> (vId, kId));
		vertexKfMapInv[kId] = vKf;
		vId ++;
	}

	for (mpid &mid: mappointList) {

		g2o::VertexSBAPointXYZ *vMp = new g2o::VertexSBAPointXYZ();
		MapPoint *mp = orgMap->mappoint(mid);
		vMp->setEstimate(mp->getPosition());
		vMp->setMarginalized(true);
		vMp->setId(vId);
		optimizer.addVertex(vMp);
		vertexMpMap.insert(pair<oid, mpid> (vId, mid));
		vertexMpMapInv[mid] = vMp;
		vId ++;

		// Edges
		for (auto &kfIds: orgMap->getRelatedKeyFrames(mid)) {

			// Get map point's projection in this particular keyframe
//			KeyFrame *rkf = orgMap->keyframe(kfIds);
//			map<mpid,kpid> &listKpt = orgMap->allMapPointsAtKeyFrame(kfIds);
//			oid ekpid = orgMap->getKeyPointId(kfIds, mid);
//			cv::Point2f p2D = rkf->getKeyPointAt(ekpid).pt;
			cv::Point2f p2D = orgMap->keyframe(kfIds)
				->getKeyPointAt(orgMap
					->getKeyPointId(kfIds, mid))
						.pt;

			g2o::EdgeSE3ProjectXYZ *edge = new g2o::EdgeSE3ProjectXYZ();
			edge->setVertex(0, vMp);
			edge->setVertex(1, vertexKfMapInv[kfIds]);
			edge->setMeasurement(Vector2d(p2D.x, p2D.y));
			// XXX: Doubtful
			edge->setInformation(Matrix2d::Identity());

			edge->fx = orgMap->getCameraParameters().fx;
			edge->fy = orgMap->getCameraParameters().fy;
			edge->cx = orgMap->getCameraParameters().cx;
			edge->cy = orgMap->getCameraParameters().cy;

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
		fromSE3Quat(kfPoseSE3, kf->getPosition(), kf->getOrientation());
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
