
#include "kf_tracking.h"


KFTracking::KFTracking() {

}

Matrix KFTracking::getX() {

	return x;
}

void KFTracking::initialize(const vector<double> &params) {

	double init_x 	= params.at(0);
	double init_y 	= params.at(1);
	double std_x  	= params.at(2);
	double std_y  	= params.at(3);
	double dt 	 		= params.at(4);
	double process_noise = params.at(5);

	this->dt = dt;

	this->x = { 4, 1, { init_x, 
											init_y, 
											0, 
											0 } };

	this->A = { 4, 4, {  1, 0, this->dt, 	0,
											 0, 1, 0, 		 		this->dt,
											 0, 0, 1, 		 		0,
											 0, 0, 0,		 			1 } };

	this->H = { 2, 4, { 1, 0, 0, 0,
											0, 1, 0, 0 } };

	this->Q = { 4, 4, {  std::pow(this->dt, 4)/4, 	0,												std::pow(this->dt, 3)/2,	0,
											 0,													std::pow(this->dt, 4)/4,	0,												std::pow(this->dt, 3)/2,
											 std::pow(this->dt, 3)/2,		0,												std::pow(this->dt, 2),		0,
											 0,													std::pow(this->dt, 3)/2, 	0, 												std::pow(this->dt, 2) 	} };
	Q = Q * std::pow(process_noise, 2);

	this->R = { 2, 2, { std::pow(std_x, 2),  	0,
											0,										std::pow(std_y, 2) }	};

	this->P = { 4, 4, { 1, 0, 0, 0,
											0, 1, 0, 0,
											0, 0, 1, 0,
											0, 0, 0, 1 } };

											
	this->I = { 4, 4, {  1, 0, 0, 0,
											 0, 1, 0, 0,
											 0, 0, 1, 0,
											 0, 0, 0, 1 } };
}


void KFTracking::predict() {

	// x = A @ x
	x = A * x;

	// P = A @ P @ A.T + Q
	P = A * P * A.transpose() + Q;
}


void KFTracking::update(Matrix& z) {
	
	// S = H @ P @ H.T + R
	Matrix S = H * P * H.transpose() + R;

	// Inverse of S
	double a = S.at(0, 0);
	double b = S.at(0, 1);
	double c = S.at(1, 0);
	double d = S.at(1, 1);
	double det = (a * d) - (b * c);
	det = 1 / det;
	Matrix SI { 2, 2, { d, -b, 
										 -c, a } };
	SI = SI * det;
										 
	// K = P @ H.T @ inv(S)
	Matrix K = P * H.transpose() * SI;
										 
	// x = x + K @ (z - H @ x)
	x = x + K * ( z - H * x );

	// P = (I - (K @ H)) @ P
	P = ( I - (K * H) ) * P;
}





