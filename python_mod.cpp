/*
 * python_mod.cpp
 *
 *  Created on: Jul 17, 2018
 *      Author: sujiwo
 */


#include <string>
#include <boost/python.hpp>
#include <numpy/arrayobject.h>
#include "VMap.h"
#include "MapPoint.h"
#include "KeyFrame.h"


using namespace boost::python;
using namespace std;
using namespace Eigen;


boost::python::object Eigen2Numpy
(const MatrixXd &M)
{
//	double *data = const_cast<double*>(M.data());
//	npy_intp dsize[2] = {M.rows(), M.cols()};
//	PyObject *pyobjx = PyArray_New(&PyArray_Type, 2, dsize, NPY_DOUBLE, NULL, data, 0, NPY_ARRAY_CARRAY, NULL);
//	boost::python::handle<> h(pyobjx);
//	return object(h);

//	numeric::array G;
//	G.setshape(M.rows(), M.cols());
//	return G;
}


cv::Mat Numpy2CV ()
{

}


class VMapPy : public VMap
{
public:

	bool find ()
	{
		return true;
	}


	dict info ()
	{
		dict mapinfo;
		mapinfo["numKeyframes"] = this->numOfKeyFrames();
		mapinfo["numMappoints"] = this->numOfMapPoints();
		return mapinfo;
	}


	boost::python::object
	allMapPoints ()
	{
		uint N = this->numOfMapPoints();
		MatrixXd allPts(N,3);

		for (uint i=0; i<N; i++) {
			allPts.row(i) = mappoint(i)->getPosition();
		}

		return Eigen2Numpy (allPts);
	}


	boost::python::object
	allKeyFrames ()
	{
		// XXX: Stub
	}
};


boost::python::object
testMatrix ()
{
	Matrix3d M = Matrix3d::Identity();
	return Eigen2Numpy (M);
}


string
testD ()
{
	return string("ABC");
}


BOOST_PYTHON_MODULE(photogrampy)
{
	class_ <VMapPy> ("VMap")

		.def("load", &VMapPy::load)

		.def("info", &VMapPy::info)

		.def("allMapPoints", &VMapPy::allMapPoints);

	def("testmatrix", &testMatrix);

	def ("testd", &testD);

	import_array();
}

