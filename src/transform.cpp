// #include "unit_trans.h"
#include "my_type.h"
namespace elmo{
	const double pi = 3.141592654;
}
double rad2deg(double rad)
{
	return rad * 180. / elmo::pi;
}

double deg2rad(double deg)
{
	return deg * elmo::pi / 180.;
}

double rad_s2rpm(double rad)
{
	return  rad * 30 / elmo::pi;
}

double rpm2rad_s(double rpm)
{
	return  rpm / 30 * elmo::pi;
}

double analog2vol(short a)
{
	return static_cast<double>(a);
}

