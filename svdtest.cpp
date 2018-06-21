#include <Eigen/Eigen>


using namespace Eigen;


int main (int argc, char *argv[])
{
	Matrix4d A;
	A << 1, 6, 1, 2,
		9, 5, 3, 7,
		8, 6, 4, 1,
		3, 2, 0, 6;
	return 0;
}
