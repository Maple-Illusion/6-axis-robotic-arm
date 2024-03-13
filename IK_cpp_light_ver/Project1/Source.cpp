#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <windows.h>
#include <conio.h>
#include <time.h>

///tool
#include "Inverse_Kinematics.h"


///

double origin[3];
double final[3];
double pos[3] = { 0, 0, 0 };
double R, P, Y;
double ang[6];
double org_ang[6],f_ang[6];
unsigned int instruct;
double move_t;

double tf, dt, t;
double a0[6], a1[6], a2[6], a3[6];
double Vel[6], Acc[6];

using namespace std;
using std::string;

int main() {
	////initial
	double * fp =f_ang;
	double * op = org_ang;
	double * p = ang;
	origin[0] = 520.5; origin[1] = 0; origin[2] = 834;
	final[0] = 520.5 - 95.5; final[1] = 0; final[2] = 834 - 95.5;
	IK(final[0], final[1], final[2], 0, 90, 0, fp);
	////fcn
	tf = 2;
	

	for (int i = 0;i < 6;++i) {
		a3[i] = org_ang[i];
		a0[i] = -(f_ang[i] - org_ang[i]) / (0.5*pow(tf, 3));
		a1[i] = -a0[i] * 3 * tf;
		//printf("Vel: %lf * x^2 + %lf * x\n",3*a0[i],2*a1[i]);
		//printf("Acc: %lf * x + %lf\n", 6 * a0[i], 2 * a1[i]);
	}
	////

	cout << f_ang[0] << " " << f_ang[1] << " " << f_ang[2] << " " << f_ang[3] << " " << f_ang[4] << " " << f_ang[5] << endl;
	origin[0] = final[0]; origin[1] = final[1]; origin[2] = final[2];
	
	
	////
	
	while (1) {
		if (origin[0] == final[0] && origin[1] == final[1] && origin[2] == final[2]) {
			for (int i = 0;i < 5;++i) {
				org_ang[i] = f_ang[i];
			}
			t = 0;
			dt = tf / 100;
			
			for (int i = 0; i < 6; ++i) {
				Vel[i] = 0; Acc[i] = 0;
			}
			cout << "Coordinate" << endl;
			cin >> final[0] >> final[1] >> final[2];
			cout << "RPY" << endl;
			cin >> R >> P >> Y;
			IK(final[0], final[1], final[2], R, P, Y, fp);
			for (int i = 0; i < 6; ++i) {
				a3[i] = org_ang[i];
				a0[i] = -(f_ang[i] - org_ang[i]) / (0.5*pow(tf, 3));
				a1[i] = -a0[i] * 3 * tf / 2;
				printf("Pos[%d]: %lf * x^3 + %lf * x^2 + %lf\n", i, a0[i], a1[i], a3[i]);
				printf("Vel[%d]: %lf * x^2 + %lf * x\n",i, 3 * a0[i], 2 * a1[i]);
				printf("Acc[%d]: %lf * x + %lf\n",i, 6 * a0[i], 2 * a1[i]);
				
			}
			
			cout << "Ans: " << f_ang[0] << " " << f_ang[1] << " " << f_ang[2] << " " << f_ang[3] << " " << f_ang[4] << " " << f_ang[5] << endl;

		}
		
			
		
		for (int j=0 ;j < 5 ;++j) {
			ang[j] = a0[j] * pow(t, 3) + a1[j] * pow(t, 2) + a3[j];
			Vel[j] = 3 * a0[j] * pow(t, 2) + 2 * a1[j] * t;
			Acc[j] = 6 * a0[j] * t + 2 * a1[j];

		}
		cout << ang[0] << " " << ang[1] << " " << ang[2] << " " << ang[3] << " " << ang[4] << " " << ang[5] << " Time: " << t << endl;



#if 0
		cout << "Move pattern" << endl;
		cin >> instruct;
		switch (instruct){
		case 0: //reset
			pos[0] = 520.5;
			pos[1] = 0;
			pos[2] = 834;
		case 1:
			cout << "Input Pos" << endl;
			cin >> pos[0] >> pos[1] >> pos[2];
			cout << "Input RPY" << endl;
			cin >> R >> P >> Y; 
			
			
		case 2:
			cout << "Input Pos" << endl;
			cin >> pos[0] >> pos[1] >> pos[2];
			cout << "Input RPY" << endl;
			cin >> R >> P >> Y;
		case -1:
			break;
		}
#endif
		//IK(pos[0], pos[1], pos[2], R, P, Y, p);
		
		if (t < tf) {
			t = t + dt;
		}
		else {
			origin[0] = final[0]; origin[1] = final[1]; origin[2] = final[2];
			cout<< "Goat: " << ang[0] << " " << ang[1] << " " << ang[2] << " " << ang[3] << " " << ang[4] << " " << ang[5] << endl;
		}
	}


	return 0;
}

