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


g2o::SE3Quat toSE3Quat (const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation)
{ return g2o::SE3Quat(orientation, position); }


void bundle_adjustment (VMap *orgMap)
{
	vector<kfid> keyframeList = orgMap->allKeyFrames();
	vector<mpid> mappointList = orgMap->allMapPoints();

	g2o::SparseOptimizer optimizer;
	g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType> linearSolver;

	map<oid, pair<char,oid> > vertexIdMap;
	uint64 vId = 0;

	for (kfid &kid: keyframeList) {
		g2o::VertexSE3Expmap *vKf = new g2o::VertexSE3Expmap();
		KeyFrame *kf = orgMap->keyframe(kid);
		vKf->setEstimate (toSE3Quat(kf->getPosition(), kf->getOrientation()));
		vKf->setId(vId);
		optimizer.addVertex(vKf);
		vertexIdMap.insert(pair<oid, pair<char,oid> > (vId, pair<char,oid>('k', kid)));
		vId ++;
	}

	for (mpid &mid: mappointList) {
		g2o::VertexSBAPointXYZ *vMp = new g2o::VertexSBAPointXYZ();
		MapPoint *mp = orgMap->mappoint(mid);
		vMp->setEstimate(mp->getPosition());
		vMp->setMarginalized(true);
		vMp->setId(vId);
		optimizer.addVertex(vMp);
		vertexIdMap.insert(pair<oid, pair<char,oid> > (vId, pair<char,oid>('m', mid)));

		// Edges
	}
}
