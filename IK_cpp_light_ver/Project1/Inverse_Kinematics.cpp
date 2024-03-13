#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <windows.h>
#include <conio.h>

#include "Inverse_Kinematics.h"



extern double pi = acos(-1.0);
using namespace std;
using std::string;




void IK(double x, double y, double z, double R, double P, double Y,double * ANG) {
	
	////IK_variables_initialization
	double pos[3] = { 0, 0, 0 };
	double alpha, r1, r2, beta, sigma;
	double seige[8] = { 0 };
	double th[6] = { 0 ,0 ,0 ,0 ,0 ,0 }, th23;
	double ang[6] = { 0 ,0 ,0 ,0 ,0 ,0 };
	
	double DH[2][6] = { { 0, 105, 0, 146, 174,95.5 },{ 289, 200, 280, 65, 0, 0 } };
	double Origin[3] = { 520.5, 0, 834 };
	double cR, cP, cY, sR, sP, sY;
	double C[6] = { 0 }, S[6] = { 0 };
	double J[6][3] = { 0 }, J6org[3] = { 0 };
	double(*Jp)[6][3] = &J;
	double L01 = 289, L12 = sqrt(200 * 200 + 105 * 105), L23 = 280, L34 = sqrt(146 * 146 + 65 * 65), L45 = 174.0, L56 = 95.5;
	double L25, L35, L46, L66;
	double Rrpy[3][3] = { 0 }, R01[3][3] = { 0 }, R12[3][3] = { 0 }, R23[3][3] = { 0 };
	double R34[3][3] = { 0 }, R45[3][3] = { 0 }, R56[3][3] = { 0 }, R67[3][3] = { 0 }, R05[3][3] = { 0 };
	double Suc[3][3] = { 0 }, Suc2[3][3] = { 0 };
	
	////

	///input transform
	pos[0] = x;
	pos[1] = y;
	pos[2] = z;
	///

	


#if 0
	double G[2][3] = { { 1,2,3 },{ 0,0,0 } };
	double GG[2][3] = { { 0,0,0 },{ 4 ,6 ,7 } };
	double LG = c2c_lenth(&G[0][0], &GG[1][0]);
	cout << LG << endl;
#endif

	cR = cos(d2r(R));cP = cos(d2r(P));cY = cos(d2r(Y));
	sR = sin(d2r(R)); sP = sin(d2r(P)); sY = sin(d2r(Y));

	J[5][0] = pos[0]; J[5][1] = pos[1]; J[5][2] = pos[2];//J6
	J[4][0] = pos[0] - 95.5*cP*cY; J[4][1] = pos[1] - 95.5*cP*sY; J[4][2] = pos[2] + 95.5*sP;//J5

	th[0] = r2d(atan(J[4][1] / J[4][0]));
	J[1][0] = 105 * cos(d2r(th[0])); J[1][1] = 105 * sin(d2r(th[0])); J[1][2] = 289.0 + 200.0;

	L25 = c2c_lenth(&J[1][0], &J[4][0]);
	//cout << L25 << endl;
	alpha = pi - atan(DH[1][3] / DH[0][3]);
	//cout << r2d(alpha) << endl;
	L35 = sqrt(L45*L45 + L34*L34 - 2 * L45*L34*cos(alpha));
	if (((L35*L35 - L25*L25 - L23*L23) / (-2 * L25*L23)) >= 1) {
		seige[0] = 0.999999;
	}
	else if (((L35*L35 - L25*L25 - L23*L23) / (-2 * L25*L23)) <= -1) {
		seige[0] = -0.999999;
	}
	else {
		seige[0] = (L35*L35 - L25*L25 - L23*L23) / (-2 * L25*L23);
	}
	sigma = r2d(acos(seige[0]));
	

	if ((J[4][2] - J[1][2]) / L25 >= 1) {
		seige[1] = 0.999999;
	}
	else if ((J[4][2] - J[1][2]) / L25 <= -1) {
		seige[1] = -0.999999;
	}
	else {
		seige[1] = (J[4][2] - J[1][2]) / L25;
	}
	beta = r2d(acos(seige[1]));
	

	if (L34*sin(alpha) / L35 >= 1) {
		seige[2] = 0.999999;
	}
	else if (L34*sin(alpha) / L35 <= -1) {
		seige[2] = -0.999999;
	}
	else {
		seige[2] = L34*sin(alpha) / L35;
	}
	r1 = r2d(asin(seige[2]));


	if ((L23*L23 - L35*L35 - L25*L25) / (-2 * L35*L25) >= 1) {
		seige[3] = 0.999999;
	}
	else if ((L23*L23 - L35*L35 - L25*L25) / (-2 * L35*L25) <= -1) {
		seige[3] = -0.999999;
	}
	else {
		seige[3] = (L23*L23 - L35*L35 - L25*L25) / (-2 * L35*L25);
	}
	r2 = r2d(acos(seige[3]));
	th23 = beta + r1 + r2 - 90;
	th[1] = beta - sigma;//ang2
	th[2] = th23 - th[1];//ang3


	J[2][0] = J[1][0] + L23*sin(d2r(th[1]))*cos(d2r(th[0])); J[2][1] = J[1][1] + L23*sin(d2r(th[1]))*sin(d2r(th[0])); J[2][2] = J[1][2] + 280 * cos(d2r(th[1]));
	J[3][0] = J[2][0] + 146 * cos(d2r(th[1] + th[2]))*cos(d2r(th[0])) + 65 * sin(d2r(th[1] + th[2])*cos(d2r(th[0])));
	J[3][1] = J[2][1] + 146 * cos(d2r(th[1] + th[2]))*sin(d2r(th[0])) + 65 * sin(d2r(th[1] + th[2])*sin(d2r(th[0])));
	J[3][2] = J[2][2] - 146 * sin(d2r(th[1] + th[2])) + 65 * cos(d2r(th[1] + th[2]));
	L46 = c2c_lenth(&J[3][0], &J[5][0]);

	if ((L46*L46 - L45*L45 - L56*L56) / (-2 * L45*L56) >= 1) {
		seige[4] = 0.999999;
	}
	else if ((L46*L46 - L45*L45 - L56*L56) / (-2 * L45*L56) <= -1) {
		seige[4] = -0.999999;
	}
	else {
		seige[4] = (L46*L46 - L45*L45 - L56*L56) / (-2 * L45*L56);
	}
	th[4] = 180 - r2d(acos(seige[4]));


	J6org[0] = J[4][0] + L56*cos(d2r(th[1] + th[2] + th[4]))*cos(d2r(th[0]));
	J6org[1] = J[4][1] + L56*cos(d2r(th[1] + th[2] + th[4]))*sin(d2r(th[0]));
	J6org[2] = J[4][2] - L56*sin(d2r(th[1] + th[2] + th[4]));
	L66 = c2c_lenth(&J6org[0], &J[5][0]);

	if ((L66*L66 - 2 * L56*sin(d2r(th[4]))*L56*sin(d2r(th[4]))) / (-2 * L56*sin(d2r(th[4]))*L56*sin(d2r(th[4]))) >= 1) {
		seige[5] = 0.999999;
	}
	else if ((L66*L66 - 2 * L56*sin(d2r(th[4]))*L56*sin(d2r(th[4]))) / (-2 * L56*sin(d2r(th[4]))*L56*sin(d2r(th[4]))) <= -1) {
		seige[5] = -0.999999;
	}
	else {
		seige[5] = (L66*L66 - 2 * L56*sin(d2r(th[4]))*L56*sin(d2r(th[4]))) / (-2 * L56*sin(d2r(th[4]))*L56*sin(d2r(th[4])));
	}

	th[3] = -r2d(acos(seige[5]));
	

	if (R == 0 && P == 0 && Y == 0 && c2c_lenth(&Origin[0], &pos[0]) <= 0.01) {
		th[3] = 0.0;
	}
	if (abs(th[3]) >= 179.9999) {
		if (abs(th[3]) - 179.9999 <= 0.0001) {
			th[4] = -th[4];
			th[3] = 0;
		}
		else if (th[3] > 0) {
			th[4] = -th[4];
			th[3] = abs(th[3]) - 179.9999;
		}
		else if (th[3] < 0) {
			th[4] = -th[4];
			th[3] = th[3] + 179.9999;
		}
	}

	//th[4] = -th[4];

	C[0] = cos(d2r(th[0])); C[1] = cos(d2r(th[1])); C[2] = cos(d2r(th[2]));
	C[3] = cos(d2r(th[3])); C[4] = cos(d2r(th[4])); C[5] = cos(d2r(th[5]));
	S[0] = sin(d2r(th[0])); S[1] = sin(d2r(th[1])); S[2] = sin(d2r(th[2]));
	S[3] = sin(d2r(th[3])); S[4] = sin(d2r(th[4])); S[5] = sin(d2r(th[5]));

	R01[0][0] = 1; R01[1][1] = 1; R01[2][2] = 1;

	R12[0][0] = C[0]; R12[0][1] = -S[0]; R12[1][0] = S[0]; R12[1][1] = C[0]; R12[2][2] = 1;//RZ
	R23[0][0] = C[1]; R23[0][2] = S[1]; R23[2][0] = -S[1]; R23[2][2] = C[1]; R23[1][1] = 1;//RY
	R34[0][0] = C[2]; R34[0][2] = S[2]; R34[2][0] = -S[2]; R34[2][2] = C[2]; R34[1][1] = 1;//RY
	R45[1][1] = C[3]; R45[1][2] = -S[3]; R45[2][1] = S[3]; R45[2][2] = C[3]; R45[0][0] = 1;//RX
	R56[0][0] = C[4]; R56[0][2] = S[4]; R56[2][0] = -S[4]; R56[2][2] = C[4]; R56[1][1] = 1;//RY
	R67[1][1] = C[5]; R67[1][2] = -S[5]; R67[2][1] = S[5]; R67[2][2] = C[5]; R67[0][0] = 1;//RX

	Rrpy[0][0] = cY*cP; Rrpy[0][1] = -sY*cR + cY*sP*sR; Rrpy[0][2] = sY*sR + cY*sP*cR;
	Rrpy[1][0] = sY*cP; Rrpy[1][1] = cY*cR + sY*sP*sR; Rrpy[1][2] = -cY*sR + sY*sP*cR;
	Rrpy[2][0] = -sP; Rrpy[2][1] = cP*sR; Rrpy[2][2] = cP*cR;

	//cout << th[0] << " " << th[1] << " " << th[2] << " " << th[3] << " " << th[4] << " " << th[5] << endl;


#if 1
	m2p(R01, R12, Suc);
	m2p(Suc, R23, Suc2);//自己去自訂函數改矩陣大小
	m2p(Suc2, R34, Suc);
	m2p(Suc, R45, Suc2);
	m2p(Suc2, R56, R05);
	m2t(R05, Suc);//R05 transpose
#endif
	m2p(Suc, Rrpy, Suc2);


	//cout << Suc2[1][2] << endl;
	th[5] = r2d(asin(-Suc2[1][2]));


	//cout << th[0] << " " << th[1] << " " << th[2] << " " << th[3] << " " << th[4] << " " << th[5] << endl;
	*(ANG) = th[0];
	*(ANG+1) = th[1];
	*(ANG+2) = th[2];
	*(ANG+3) = th[3];
	*(ANG+4) = th[4];
	*(ANG+5) = th[5];

}


//////////自訂函式
double m2p(double A[3][3], double B[3][3], double C[3][3]) {
	memset(C, 0, sizeof(double) * 9);
	for (int i = 0; i<3; ++i) {
		for (int j = 0; j<3; ++j) {
			for (int k = 0; k<3; ++k) {
				C[i][j] += A[i][k] * B[k][j];
			}
		}
	}
	return 0;

}
double m2t(double A[3][3], double B[3][3]) {

	for (int i = 0;i < 3;++i) {
		for (int j = 0;j < 3;++j) {
			B[j][i] = A[i][j];
		}
	}
	return 0;

}

void RX(double A[4][4], double ang) {
	double AA[4][4] = { { 1,0,0,0 },{ 0,cos(ang),-sin(ang),0 },{ 0,sin(ang),cos(ang),0 },{ 0,0,0,1 } };
	for (int i = 0; i<4; ++i) {
		for (int j = 0; j<4; ++j) {
			A[i][j] = AA[i][j];
		}
	}
}

void RY(double A[4][4], double ang) {
	double AA[4][4] = { { cos(ang),0,sin(ang),0 },{ 0,1,0,0 },{ -sin(ang),0,cos(ang),0 },{ 0,0,0,1 } };
	for (int i = 0; i<4; ++i) {
		for (int j = 0; j<4; ++j) {
			A[i][j] = AA[i][j];
		}
	}
}

void RZ(double A[4][4], double ang) {
	double AA[4][4] = { { cos(ang),-sin(ang),0,0 },{ sin(ang),cos(ang),0,0 },{ 0,0,1,0 },{ 0,0,0,1 } };
	for (int i = 0; i<4; ++i) {
		for (int j = 0; j<4; ++j) {
			A[i][j] = AA[i][j];
		}
	}
}

double r2d(double G) {
	double A = 0.0;
	A = G * 180.0 / pi;
	return A;
}
double d2r(double G) {
	double A = 0.0;
	A = G *pi / 180.0;
	return A;
}

void placement(double A, double B, double C, double AA[4][4]) {
	AA[0][3] = A;
	AA[1][3] = B;
	AA[2][3] = C;

}

double c2c_lenth(double * A, double * B) {
	double lenth = 0;
	lenth = sqrt(((*A) - (*B))*((*A) - (*B)) + ((*(A + 1)) - (*(B + 1)))*((*(A + 1)) - (*(B + 1))) + ((*(A + 2)) - (*(B + 2)))*((*(A + 2)) - (*(B + 2))));

	return lenth;
}
