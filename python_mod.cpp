/*
 * python_mod.cpp
 *
 *  Created on: Jul 17, 2018
 *      Author: sujiwo
 */


#include <boost/python.hpp>
#include <numpy/ndarrayobject.h>
#include "VMap.h"


using namespace boost::python;


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
		// XXX: Stub
	}

	boost::python::object
	allKeyFrames ()
	{
		// XXX: Stub
	}
};


BOOST_PYTHON_MODULE(photogrampy)
{
	class_ <VMapPy> ("VMap")
		.def("load", &VMapPy::load)
		.def("info", &VMapPy::info)
		.def("allMapPoints", &VMapPy::allMapPoints);
}

