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


g2o::SE3Quat toSE3Quat (const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation)
{ return g2o::SE3Quat(orientation, position); }


void bundle_adjustment (
	vector<KeyFrame*> &kfList,
	vector<MapPoint*> &mpList,
	map<KeyFrame*, set<MapPoint*> > kfToMp,
	map<MapPoint*, set<KeyFrame*> > mpToKf,

)
{
	g2o::SparseOptimizer optimizer;
	g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType> linearSolver;

	uint64 maxKfId = 0;

	for (auto *kf: kfList) {
		g2o::VertexSE3Expmap *vKf = new g2o::VertexSE3Expmap();
		vKf->setEstimate (toSE3Quat(kf->getPosition(), kf->getOrientation()));
		vKf->setId(kf->getId());
		optimizer.addVertex(vKf);
//		if (kf->getId() > maxKfId)
		maxKfId = (kf->getId() > maxKfId ? kf->getId() : maxKfId);
	}

	for (auto *mp: mpList) {
		g2o::VertexSBAPointXYZ *vMp = new g2o::VertexSBAPointXYZ();
		vMp->setEstimate(mp->getPosition());
		vMp->setMarginalized(true);
		vMp->setId(mp->getId() + maxKfId + 1);
		optimizer.addVertex(vMp);

		// Edges
		set<KeyFrame*> kfTargets = mpToKf[mp];
		for (auto *px: kfTargets) {

//			Vector2d obs
			g2o::EdgeSE3ProjectXYZ *edge = new g2o::EdgeSE3ProjectXYZ();
		}
	}
}
