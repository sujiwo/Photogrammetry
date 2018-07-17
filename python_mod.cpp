/*
 * python_mod.cpp
 *
 *  Created on: Jul 17, 2018
 *      Author: sujiwo
 */


#include <boost/python.hpp>
#include "VMap.h"


using namespace boost::python;


BOOST_PYTHON_MODULE(photogrampy)
{
	class_ <VMap> ("VMap")
		.def("load", &VMap::load);
}

