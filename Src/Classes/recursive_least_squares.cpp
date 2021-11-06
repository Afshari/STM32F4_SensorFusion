
#include "recursive_least_squares.h"

RecursiveLeastSquares::RecursiveLeastSquares() {

}

#ifdef USE_CMSIS_DSP

shared_ptr<vector<double>> RecursiveLeastSquares::getX() {

	shared_ptr<vector<double>> result = make_shared<vector<double>>();
	result->push_back(x.pData[0]);
	result->push_back(x.pData[1]);
	return result;
}


void RecursiveLeastSquares::initialize(const vector<double> &params) {

	float init_x_1 = params.at(0);
	float init_x_2 = params.at(1);
	float init_R   = params.at(2);


	float32_t t_x[] = { init_x_1, init_x_2 };
	std::copy(std::begin(t_x), std::end(t_x), std::begin(this->x_f32));
	arm_mat_init_f32(&this->x, 2, 1, (float32_t *)this->x_f32);


	float32_t t_R[] = { init_R };
	std::copy(std::begin(t_R), std::end(t_R), std::begin(this->R_f32));
	arm_mat_init_f32(&this->R, 1, 1, (float32_t *)this->R_f32);

	float32_t t_P[] = {  1, 0,
					 	 0, 1 };
	std::copy(std::begin(t_P), std::end(t_P), std::begin(this->P_f32));
	arm_mat_init_f32(&this->P, 2, 2, (float32_t *)this->P_f32);

}

void RecursiveLeastSquares::calculate(const vector<double> &params) {

	float param_H_1 = params.at(0);
	float param_H_2 = params.at(1);
	float param_y   = params.at(2);


	float32_t t_H[] = { param_H_1, param_H_2 };
	std::copy(std::begin(t_H), std::end(t_H), std::begin(this->H_f32));
	arm_mat_init_f32(&this->H, 1, 2, (float32_t *)this->H_f32);


	float32_t t_y[] = { param_y };
	std::copy(std::begin(t_y), std::end(t_y), std::begin(this->y_f32));
	arm_mat_init_f32(&this->y, 1, 1, (float32_t *)this->y_f32);


	arm_matrix_instance_f32 S;
	arm_matrix_instance_f32 SI;
	arm_matrix_instance_f32 HP;
	arm_matrix_instance_f32 HT;
	arm_matrix_instance_f32 HPHT;
	arm_matrix_instance_f32 PHT;
	arm_matrix_instance_f32 K;
	arm_matrix_instance_f32 Hx;
	arm_matrix_instance_f32 yHx;
	arm_matrix_instance_f32 KyHx;
	arm_matrix_instance_f32 I;
	arm_matrix_instance_f32 KH;
	arm_matrix_instance_f32 IKH;

	float S_f32[1*1];
	float HP_f32[1*2];
	float HT_f32[2*1];
	float HPHT_f32[1*1];
	float PHT_f32[2*1];
	float K_f32[2*1];
	float Hx_f32[1*1];
	float yHx_f32[1*1];
	float KyHx_f32[2*1];
	float KH_f32[2*2];
	float IKH_f32[2*2];


	std::copy(std::begin(this->init_11), std::end(this->init_11), std::begin(S_f32));
	std::copy(std::begin(this->init_21), std::end(this->init_21), std::begin(HP_f32));
	std::copy(std::begin(this->init_21), std::end(this->init_21), std::begin(HT_f32));
	std::copy(std::begin(this->init_11), std::end(this->init_11), std::begin(HPHT_f32));
	std::copy(std::begin(this->init_21), std::end(this->init_21), std::begin(PHT_f32));
	std::copy(std::begin(this->init_21), std::end(this->init_21), std::begin(K_f32));
	std::copy(std::begin(this->init_11), std::end(this->init_11), std::begin(Hx_f32));
	std::copy(std::begin(this->init_11), std::end(this->init_11), std::begin(yHx_f32));
	std::copy(std::begin(this->init_21), std::end(this->init_21), std::begin(KyHx_f32));
	std::copy(std::begin(this->init_22), std::end(this->init_22), std::begin(KH_f32));
	std::copy(std::begin(this->init_22), std::end(this->init_22), std::begin(IKH_f32));



	arm_mat_init_f32(&S, 1, 1, (float32_t *)S_f32);
	arm_mat_init_f32(&HP, 1, 2, (float32_t *)HP_f32);
	arm_mat_init_f32(&HT, 2, 1, (float32_t *)HT_f32);
	arm_mat_init_f32(&HPHT, 1, 1, (float32_t *)HPHT_f32);
	arm_mat_init_f32(&PHT, 2, 1, (float32_t *)PHT_f32);
	arm_mat_init_f32(&K, 2, 1, (float32_t *)K_f32);
	arm_mat_init_f32(&Hx, 1, 1, (float32_t *)Hx_f32);
	arm_mat_init_f32(&yHx, 1, 1, (float32_t *)yHx_f32);
	arm_mat_init_f32(&KyHx, 2, 1, (float32_t *)KyHx_f32);
	arm_mat_init_f32(&KH, 2, 2, (float32_t *)KH_f32);
	arm_mat_init_f32(&IKH, 2, 2, (float32_t *)IKH_f32);

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
	float SI_f32[1*1] = { 1/S.pData[0] };
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


shared_ptr<vector<double>> RecursiveLeastSquares::getX() {

	shared_ptr<vector<double>> result = make_shared<vector<double>>();
	result->push_back(this->x.at(0));
	result->push_back(this->x.at(1));
	return result;
}

void RecursiveLeastSquares::initialize(const vector<double> &params) {

	float init_x_1 = params.at(0);
	float init_x_2 = params.at(1);
	float init_R   = params.at(2);

	this->x.resize( 2 * 1 );
	this->R.resize( 1 * 1 );
	this->P.resize( 2 * 2 );
	
	this->x.at(0) = init_x_1;
	this->x.at(1) = init_x_2;
	this->R.at(0) = init_R;

	double t_P[] = {  1, 0,
										0, 1 };
	std::copy(std::begin(t_P), std::end(t_P), std::begin(this->P));

}

void RecursiveLeastSquares::calculate(const vector<double> &params) {

	float param_H_1 = params.at(0);
	float param_H_2 = params.at(1);
	float param_y   = params.at(2);


	vector<double> H  = { param_H_1, param_H_2 };
	vector<double> HT = { param_H_1, param_H_2 };
	vector<double> y  = { param_y };

	vector<double> PHT( 2 * 1 );
	vector<double> HP( 1 * 2 );
	vector<double> HPHT( 1 * 1 );
	vector<double> S( 1 * 1 );
	vector<double> K( 2 * 1 );

	// S = H @ P @ H.T + R
	mul(H, P, HP, 1, 2, 2);
	mul(HP, HT, HPHT, 1, 2, 1);
	add(HPHT, R, S, 1, 1);

	vector<double> SI( 1 * 1 );
	if(S[0] == 0)
		SI[0] = 1;
	else
		SI[0] = 1/S[0];

	// K = P @ H.T @ np.linalg.pinv(S)
	mul(P, HT, PHT, 2, 2, 1);
	mul(PHT, SI, K, 2, 1, 1);


	vector<double> Hx( 1 * 1 );
	vector<double> yHx( 1 * 1 );
	vector<double> KyHx( 2 * 1 );

	// x = x + K * (y - H @ x)
	mul(H, x, Hx, 1, 2, 1);
	sub(y, Hx, yHx, 1, 1);
	mul(K, yHx, KyHx, 2, 1, 1);
	add(x, KyHx, x, 2, 1);

	vector<double> KH( 2 * 2 );
	vector<double> KHP( 2 * 2 );

	// P = P - K @ H @ P
	mul(H, P, HP, 1, 2, 2);
	mul(K, HP, KHP, 2, 1, 2);
	sub(P, KHP, P, 2, 2);

}

#endif

