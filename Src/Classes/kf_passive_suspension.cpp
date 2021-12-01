
#include "kf_passive_suspension.h"

KFPassiveSuspension::KFPassiveSuspension() {

}

Matrix KFPassiveSuspension::getX() {

	return this->x;
}

void KFPassiveSuspension::initialize() {

	x = { 4, 1, { 0,
								0,
								0,
								0 } };

	H = { 1, 4, { 1, 0, 0, 0 } };
	R = { 1, 1, { 1e-6 } };

	P = { 4, 4, {
						 1,     	0,      0,      0,
             0,     	1,      0,      0,
             0,     	0,      1,      0,
             0,     	0,      0,      1 } };

	F = { 4, 4, {
							 0.9998,    0.0004,    0.0010,   -0.0010,
							 0.0002,    0.9996,    0.0000,    0.0010,
							-0.0437,   -0.0005,    0.9989,    0.0011,
							 0.3739,   -0.8856,    0.0098,    0.9897 	} };

	Q = { 4, 4, {
						 0.0000,    0.0000,   -0.0000,   -0.0000,
						 0.0000,    0.2233,   -0.0001,   -0.1276,
						-0.0000,   -0.0001,    0.0000,    0.0000,
						-0.0000,   -0.1276,    0.0000,    0.0875 	} };
	
	I = { 4, 4, {  
							 1, 0, 0, 0,
							 0, 1, 0, 0,
							 0, 0, 1, 0,
							 0, 0, 0, 1 } };
							 
}

void KFPassiveSuspension::predict() {

	// x = F @ x
	x = F * x;

  // P = F @ P @ F.T + Q
	P = F * Q * F.transpose() + Q;
}

void KFPassiveSuspension::update(const double &param) {

	// S = H @ P @ H.T + R
	Matrix S = H * P * H.transpose() + R;

	// K = P @ H.T @ inv(S)
	Matrix SI{ 1, 1, { 0 } };
	if(S.at(0, 0) == 0)
		SI = { 1, 1, { 1 } };
	else
		SI = { 1, 1, { 1/S.at(0, 0) } };
	Matrix K = P * H.transpose() * SI;

	Matrix z { 1, 1, { param } };
	
	// x = x + K @ (z - H @ x)
	x = x + K * ( z - H * x );

	// P = (I - (K @ H)) @ P
	P = ( I - (K * H) ) * P;
}







