
#include "recursive_least_squares.h"

RecursiveLeastSquares::RecursiveLeastSquares() {

	x = Matrix{ 2, 1, { 0, 0 } };
	R = Matrix{ 1, 1, { 0 } };

	P = Matrix{ 2, 2, { 1, 0,
											0, 1 } };
}

Matrix RecursiveLeastSquares::getX() {

	return x;
}

void RecursiveLeastSquares::initialize(const vector<double> &params) {

	float init_x_1 = params.at(0);
	float init_x_2 = params.at(1);
	float init_R   = params.at(2);
	
	x = { 2, 1, { init_x_1, init_x_2 } };
	R = { 1, 1, { init_R } };

	P = { 2, 2, { 1, 0,
								0, 1 } };

}

void RecursiveLeastSquares::calculate(const vector<double> &params) {

	float param_H_1 = params.at(0);
	float param_H_2 = params.at(1);
	float param_y   = params.at(2);


	Matrix H  = { 1, 2, { param_H_1, param_H_2 } };
	Matrix y  = { 1, 1, { param_y } };

	// S = H @ P @ H.T + R
	Matrix S = H * P * H.transpose() + R;

	Matrix SI{ 1, 1, { 0 } };
	if(S.at(0, 0) == 0)
		SI = { 1, 1, { 1 } };
	else
		SI = { 1, 1, { 1/S.at(0, 0) } };

	// K = P @ H.T @ np.linalg.pinv(S)
	Matrix K = P * H.transpose() * SI;

	// x = x + K * (y - H @ x)
	x = x + K * ( y - H * x );

	// P = P - K @ H @ P
	P = P - K * H * P;
}

