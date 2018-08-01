/*
 * test_tf.cpp
 *
 *  Created on: Jul 31, 2018
 *      Author: sujiwo
 */

#include <iostream>
#include <cstdio>
#include <string>
#include "utilities.h"


using namespace std;
using namespace Eigen;


int main (int argc, char *argv[])
{
	float
		roll = stod(argv[1]),
		pitch = stod(argv[2]),
		yaw = stod(argv[3]);

	Quaterniond q = fromRPY(roll, pitch, yaw);
	printf ("%f, %f, %f, %f\n", q.x(), q.y(), q.z(), q.w());
}
