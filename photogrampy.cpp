/*
 * python_mod.cpp
 *
 *  Created on: Jul 17, 2018
 *      Author: sujiwo
 */


#include <boost/python.hpp>
#include <Eigen/Eigen>
#include "VMap.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "Frame.h"
#include "pymat.h"


using namespace boost::python;
using namespace std;
using namespace Eigen;


static NDArrayConverter matcvt;


template<typename _Tp, int _rows>
void copyEigenVector2cv (const Eigen::Matrix<_Tp,_rows,1> &v, cv::Mat &m)
{
	assert(m.type() == cv::DataType<_Tp>::type);
	cv::Mat __(v.rows(), 1, cv::DataType<_Tp>::type, (void*)v.data(), v.stride()*sizeof(_Tp));
	__.copyTo(m);
}


class VMapPy : public VMap
{
public:

	kfid find (PyObject *inp)
	{
		cv::Mat minp = matcvt.toMat(inp);
		Frame f();
		return 0;
	}


	dict info ()
	{
		dict mapinfo;
		mapinfo["numKeyframes"] = this->numOfKeyFrames();
		mapinfo["numMappoints"] = this->numOfMapPoints();
		return mapinfo;
	}


	PyObject*
	allMapPoints ()
	{
		uint N = this->numOfMapPoints();
		cv::Mat allPts(N, 3, CV_64F);

		for (mpid i: this->getMapPointList()) {
			Vector3d p = this->mappoint(i)->getPosition();
//			cv::Mat r = allPts.row(i);
//			copyEigenVector2cv(p, r);
			allPts.row(i).at<double>(0) = p.x();
			allPts.row(i).at<double>(1) = p.y();
			allPts.row(i).at<double>(2) = p.z();
		}

		return matcvt.toNDArray(allPts);
	}


	PyObject*
	allKeyFrames ()
	{
		uint N = this->numOfKeyFrames();
		cv::Mat allKfs(N, 7, CV_64F);

		for (kfid k: this->getKeyFrameList()) {
			Vector3d p = this->keyframe(k)->getPosition();
			Quaterniond q = this->keyframe(k)->getOrientation();
			allKfs.row(k).col(0) = p.x();
			allKfs.row(k).col(1) = p.y();
			allKfs.row(k).col(2) = p.z();
			allKfs.row(k).col(3) = q.x();
			allKfs.row(k).col(4) = q.y();
			allKfs.row(k).col(5) = q.z();
			allKfs.row(k).col(6) = q.w();
		}

		return matcvt.toNDArray(allKfs);
	}
};


void
test_read (PyObject *arr)
{
	cv::Mat x = matcvt.toMat(arr);
	cerr << x << endl;
}


PyObject*
test_eye()
{
	cv::Mat M = cv::Mat::eye(3,3,CV_64F);
	cerr << M << endl;
	return matcvt.toNDArray(M);
}


BOOST_PYTHON_MODULE(photogrampy)
{
	Py_Initialize();
	import_array();
//	import_ufunc();

	class_ <VMapPy> ("VMap")

		.def("load", &VMapPy::load)

		.def("info", &VMapPy::info)

		.def("allMapPoints", &VMapPy::allMapPoints)

		.def("allKeyFrames", &VMapPy::allKeyFrames);

	def("test_read", &test_read);

	def("test_eye", &test_eye);
}

