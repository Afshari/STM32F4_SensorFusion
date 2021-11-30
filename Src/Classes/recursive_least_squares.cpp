
#include "recursive_least_squares.h"

#ifdef USE_CMSIS_DSP

RecursiveLeastSquares::RecursiveLeastSquares() {

}

shared_ptr<vector<double>> RecursiveLeastSquares::getX() {

	return make_shared<vector<double>>( x.pData[0], x.pData[1] );
}


void RecursiveLeastSquares::initialize(const vector<double> &params) {

	float init_x_1 = params.at(0);
	float init_x_2 = params.at(1);
	float init_R   = params.at(2);

	this->x_f32 = { init_x_1, init_x_2 };
	arm_mat_init_f32(&this->x, 2, 1, (float32_t *)this->x_f32.data());

	this->R_f32 = { init_R };
	arm_mat_init_f32(&this->R, 1, 1, (float32_t *)this->R_f32.data());

	this->P_f32 = {  1, 0,
									 0, 1 };
	arm_mat_init_f32(&this->P, 2, 2, (float32_t *)this->P_f32.data());
}

void RecursiveLeastSquares::calculate(const vector<double> &params) {

	float param_H_1 = params.at(0);
	float param_H_2 = params.at(1);
	float param_y   = params.at(2);

	array<float32_t, 2> t_H = { param_H_1, param_H_2 };
	arm_mat_init_f32(&this->H, 1, 2, t_H.data());


	array<float32_t, 1> t_y = { param_y };
	arm_mat_init_f32(&this->y, 1, 1, t_y.data());
	

	S_f32 			= { 0 };
	HP_f32 			= { 0, 0 };
	HT_f32			= { 0, 0 };
	HPHT_f32 		= { 0 };
	PHT_f32			= { 0, 0 };
	K_f32				= { 0 , 0 };
	Hx_f32			= { 0 };
	yHx_f32			= { 0 };
	KyHx_f32		= { 0, 0 };
	KH_f32			= { 0, 0, 0, 0 };
	IKH_f32			= { 0, 0, 0, 0 };


	arm_mat_init_f32(&S, 1, 1, (float32_t *)S_f32.data());
	arm_mat_init_f32(&HP, 1, 2, (float32_t *)HP_f32.data());
	arm_mat_init_f32(&HT, 2, 1, (float32_t *)HT_f32.data());
	arm_mat_init_f32(&HPHT, 1, 1, (float32_t *)HPHT_f32.data());
	arm_mat_init_f32(&PHT, 2, 1, (float32_t *)PHT_f32.data());
	arm_mat_init_f32(&K, 2, 1, (float32_t *)K_f32.data());
	arm_mat_init_f32(&Hx, 1, 1, (float32_t *)Hx_f32.data());
	arm_mat_init_f32(&yHx, 1, 1, (float32_t *)yHx_f32.data());
	arm_mat_init_f32(&KyHx, 2, 1, (float32_t *)KyHx_f32.data());
	arm_mat_init_f32(&KH, 2, 2, (float32_t *)KH_f32.data());
	arm_mat_init_f32(&IKH, 2, 2, (float32_t *)IKH_f32.data());

	// I = eye(2, 2);
	float32_t t_I[] = {  1, 0,
											 0, 1 };
	arm_mat_init_f32(&I, 2, 2, (float32_t *)t_I);

	//	HT = H.transpose()
	arm_status status = arm_mat_trans_f32(&this->H, &HT);

	// S = H @ P @ H.T + R
	status = arm_mat_mult_f32(&this->H, &this->P, &HP);
	status = arm_mat_mult_f32(&HP, &HT, &HPHT);
	status = arm_mat_add_f32(&HPHT, &this->R, &S);

	// K = P @ H.T @ inv(S)
	float32_t SI_f32[1*1] = { 1/S.pData[0] };
	arm_mat_init_f32(&SI, 1, 1, (float32_t *)SI_f32);
	status = arm_mat_mult_f32(&this->P, &HT, &PHT);
	status = arm_mat_mult_f32(&PHT, &SI, &K);


	// x = x + K @ (z - H @ x)
	arm_mat_mult_f32(&this->H, &this->x, &Hx);
	status = arm_mat_sub_f32(&y, &Hx, &yHx);
	status = arm_mat_mult_f32(&K, &yHx, &KyHx);
	status = arm_mat_add_f32(&this->x, &KyHx, &this->x);


	// P = (I - (K @ H)) @ P
	status = arm_mat_mult_f32(&K, &this->H, &KH);
	status = arm_mat_sub_f32(&I, &KH, &IKH);
	status = arm_mat_mult_f32(&IKH, &this->P, &this->P);

}

#else

RecursiveLeastSquares::RecursiveLeastSquares() {

	this->x = Matrix{ 2, 1, { 0, 0 } };
	this->R = Matrix{ 1, 1, { 0 } };

	this->P = Matrix{ 2, 2, { 1, 0,
														0, 1 } };
}

shared_ptr<vector<double>> RecursiveLeastSquares::getX() {

	return make_shared<vector<double>>( x.at(0, 0), x.at(1, 0) );
//	shared_ptr<vector<double>> result = make_shared<vector<double>>();
//	result->push_back(this->x.at(0, 0));
//	result->push_back(this->x.at(1, 0));
//	return result;
}

void RecursiveLeastSquares::initialize(const vector<double> &params) {

	float init_x_1 = params.at(0);
	float init_x_2 = params.at(1);
	float init_R   = params.at(2);
	
	this->x = { 2, 1, { init_x_1, init_x_2 } };
	this->R = { 1, 1, { init_R } };

	this->P = { 2, 2, { 1, 0,
											0, 1 } };

}

void RecursiveLeastSquares::calculate(const vector<double> &params) {

	float param_H_1 = params.at(0);
	float param_H_2 = params.at(1);
	float param_y   = params.at(2);


	Matrix H  = { 1, 2, { param_H_1, param_H_2 } };
	// Matrix HT = { param_H_1, param_H_2 };
	Matrix y  = { 1, 1, { param_y } };

//	vector<double> PHT( 2 * 1 );
//	vector<double> HP( 1 * 2 );
//	vector<double> HPHT( 1 * 1 );
//	vector<double> S( 1 * 1 );
//	vector<double> K( 2 * 1 );

	// S = H @ P @ H.T + R
	Matrix S = H * P * H.transpose() + R;
//	mul(H, P, HP, 1, 2, 2);
//	mul(HP, HT, HPHT, 1, 2, 1);
//	add(HPHT, R, S, 1, 1);

	Matrix SI{ 1, 1, { 0 } };
	if(S.at(0, 0) == 0)
		SI = { 1, 1, { 1 } };
	else
		SI = { 1, 1, { 1/S.at(0, 0) } };

	// K = P @ H.T @ np.linalg.pinv(S)
	Matrix K = P * H.transpose() * SI;
//	mul(P, HT, PHT, 2, 2, 1);
//	mul(PHT, SI, K, 2, 1, 1);


//	vector<double> Hx( 1 * 1 );
//	vector<double> yHx( 1 * 1 );
//	vector<double> KyHx( 2 * 1 );

	// x = x + K * (y - H @ x)
		this->x = this->x + K * ( y - H * this->x );
//	mul(H, x, Hx, 1, 2, 1);
//	sub(y, Hx, yHx, 1, 1);
//	mul(K, yHx, KyHx, 2, 1, 1);
//	add(x, KyHx, x, 2, 1);

//	vector<double> KH( 2 * 2 );
//	vector<double> KHP( 2 * 2 );

	// P = P - K @ H @ P
	this->P = this->P - K * H * this->P;
//	mul(H, P, HP, 1, 2, 2);
//	mul(K, HP, KHP, 2, 1, 2);
//	sub(P, KHP, P, 2, 2);

}

#endif

