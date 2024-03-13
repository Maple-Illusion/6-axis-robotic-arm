#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <windows.h>
#include <conio.h>

#include "Inverse_Kinematics.h"
using namespace std;
using std::string;

double trace_minimus(double*AAA, double * BBB, double R, double P, double Y, double time) {

	double origin[3] = { *(AAA), *(AAA + 1), *(AAA + 2) };
	double final[3] = { *(BBB), *(BBB + 1), *(BBB + 2) };

	return 1;

}
double trace_xyz(double*AAA, double * BBB,double R,double P,double Y, double time,double * p) {
	/// x to y to z
	double origin[3] = { *(AAA), *(AAA + 1), *(AAA + 2) };
	double final[3] = { *(BBB), *(BBB + 1), *(BBB + 2) };
	double dt = time / 100;
	double x = origin[0] - final[0]; double dx = x / dt;
	double y = origin[1] - final[1]; double dy = y / dt;
	double z = origin[2] - final[2]; double dz = z / dt;

	return 1;
}