#include "kf_tracking.h"


KFTracking::KFTracking() {

}

#ifdef USE_CMSIS_DSP

void KFTracking::initialize(const std::vector<double> &params) {

	float init_x = params.at(0);
	float init_y = params.at(1);
	float std_x  = params.at(2);
	float std_y  = params.at(3);
	float dt 	 	 = params.at(4);
	float process_noise = params.at(5);


	this->dt = dt;

	//	x = [init_x, init_y, 0, 0]
	float32_t t_x[] = { init_x, init_y, 0, 0 };
	std::copy(std::begin(t_x), std::end(t_x), std::begin(this->x_f32));
	arm_mat_init_f32(&this->x, 4, 1, (float32_t *)this->x_f32);


	//	A = [1, 	0,   	dt, 	0,
	//	     0, 	1,  	0,   	dt,
	//	     0, 	0, 		1, 		0,
	//	     0, 	0, 		0, 		1]
	float32_t t_A[] = {  1, 0, this->dt, 0,
					 	 0, 1, 0, 		this->dt,
						 0, 0, 1, 		0,
						 0, 0, 0,		1 };
	std::copy(std::begin(t_A), std::end(t_A), std::begin(this->A_f32));
	arm_mat_init_f32(&this->A, 4, 4, (float32_t *)this->A_f32);
	//	arm_mat_scale_f32(&this->A, 2, &this->A);


	//	H = [1, 0, 0, 0,
	//		 0, 1, 0, 0]
	float32_t t_H[] = {  1, 0, 0, 0,
					 	 0, 1, 0, 0 };
	std::copy(std::begin(t_H), std::end(t_H), std::begin(this->H_f32));
	arm_mat_init_f32(&this->H, 2, 4, (float32_t *)this->H_f32);



	//	Q =	  [(dt**4)/4, 		0, 				(dt**3)/2, 		 0],
	//		  	[0, 				(dt**4)/4, 		 0, 			(dt**3)/2],
	//	      [(dt**3)/2, 		0, 				 dt**2, 		 0],
	//	      [0, 				(dt**3)/2, 		 0, 	 		 dt**2]]) * std_acc**2
	float32_t t_Q[] = {   
		(float)pow(this->dt, 4)/4, 	0,														(float)pow(this->dt, 3)/2,		0,
		0,													(float)pow(this->dt, 4)/4,		0,														(float)pow(this->dt, 3)/2,
		(float)pow(this->dt, 3)/2,	0,														(float)pow(this->dt, 2),			0,
		0,													(float)pow(this->dt, 3)/2, 		0, 														(float)pow(this->dt, 2) 	};
	std::copy(std::begin(t_Q), std::end(t_Q), std::begin(this->Q_f32));
	arm_mat_init_f32(&this->Q, 4, 4, (float32_t *)this->Q_f32);
	arm_mat_scale_f32(&this->Q, std::pow(process_noise, 2), &this->Q);



	//	R = [x_std_meas**2,		0],
	//	    [0, 				y_std_meas**2]])
	float32_t t_R[] = {  (float)pow(std_x, 2),  	0,
					 	 0,				 (float)pow(std_y, 2)	};
	std::copy(std::begin(t_R), std::end(t_R), std::begin(this->R_f32));
	arm_mat_init_f32(&this->R, 2, 2, (float32_t *)this->R_f32);


	//	P = eye(4, 4);
	float32_t t_P[] = {  1, 0, 0, 0,
					 	 0, 1, 0, 0,
						 0, 0, 1, 0,
						 0, 0, 0, 1 };
	std::copy(std::begin(t_P), std::end(t_P), std::begin(this->P_f32));
	arm_mat_init_f32(&this->P, 4, 4, (float32_t *)this->P_f32);

}


void KFTracking::predict() {

	// x = A @ x
	arm_status status = arm_mat_mult_f32(&this->A, &this->x, &this->x);

	// P = A @ P @ A.T + Q
	arm_matrix_instance_f32 AT;
	arm_matrix_instance_f32 AP;
	arm_matrix_instance_f32 APAT;

	float AT_f32[4*4];
	float AP_f32[4*4];
	float APAT_f32[4*4];

	std::copy(std::begin(this->init_44), std::end(this->init_44), std::begin(AT_f32));
	std::copy(std::begin(this->init_44), std::end(this->init_44), std::begin(AP_f32));
	std::copy(std::begin(this->init_44), std::end(this->init_44), std::begin(APAT_f32));

	arm_mat_init_f32(&AT, 4, 4, (float32_t *)AT_f32);
	arm_mat_init_f32(&AP, 4, 4, (float32_t *)AP_f32);
	arm_mat_init_f32(&APAT, 4, 4, (float32_t *)APAT_f32);


	// AT = A.transpose()
	status = arm_mat_trans_f32(&this->A, &AT);

	status = arm_mat_mult_f32(&this->A, &this->P, &AP);
	status = arm_mat_mult_f32(&AP, &AT, &APAT);
	status = arm_mat_add_f32(&APAT, &Q, &this->P);

}


void KFTracking::update(const vector<double> &t_z) {

	arm_matrix_instance_f32 S;
	arm_matrix_instance_f32 HP;
	arm_matrix_instance_f32 HT;
	arm_matrix_instance_f32 HPHT;
	arm_matrix_instance_f32 PHT;
	arm_matrix_instance_f32 K;
	arm_matrix_instance_f32 Hx;
	arm_matrix_instance_f32 zHx;
	arm_matrix_instance_f32 KzHx;
	arm_matrix_instance_f32 I;
	arm_matrix_instance_f32 KH;
	arm_matrix_instance_f32 IKH;
	arm_matrix_instance_f32 z;

	float S_f32[2*2];
	float HP_f32[2*4];
	float HT_f32[4*2];
	float HPHT_f32[2*2];
	float PHT_f32[4*2];
	float K_f32[4*2];
	float Hx_f32[2*1];
	float zHx_f32[2*1];
	float KzHx_f32[4*1];
	float KH_f32[4*4];
	float IKH_f32[4*4];


	std::copy(std::begin(this->init_22), std::end(this->init_22), std::begin(S_f32));
	std::copy(std::begin(this->init_42), std::end(this->init_42), std::begin(HP_f32));
	std::copy(std::begin(this->init_42), std::end(this->init_42), std::begin(HT_f32));
	std::copy(std::begin(this->init_22), std::end(this->init_22), std::begin(HPHT_f32));
	std::copy(std::begin(this->init_42), std::end(this->init_42), std::begin(PHT_f32));
	std::copy(std::begin(this->init_42), std::end(this->init_42), std::begin(K_f32));
	std::copy(std::begin(this->init_21), std::end(this->init_21), std::begin(Hx_f32));
	std::copy(std::begin(this->init_21), std::end(this->init_21), std::begin(zHx_f32));
	std::copy(std::begin(this->init_41), std::end(this->init_41), std::begin(KzHx_f32));
	std::copy(std::begin(this->init_44), std::end(this->init_44), std::begin(KH_f32));
	std::copy(std::begin(this->init_44), std::end(this->init_44), std::begin(IKH_f32));



	arm_mat_init_f32(&S, 2, 2, (float32_t *)S_f32);
	arm_mat_init_f32(&HP, 2, 4, (float32_t *)HP_f32);
	arm_mat_init_f32(&HT, 4, 2, (float32_t *)HT_f32);
	arm_mat_init_f32(&HPHT, 2, 2, (float32_t *)HPHT_f32);
	arm_mat_init_f32(&PHT, 4, 2, (float32_t *)PHT_f32);
	arm_mat_init_f32(&K, 4, 2, (float32_t *)K_f32);
	arm_mat_init_f32(&Hx, 2, 1, (float32_t *)Hx_f32);
	arm_mat_init_f32(&zHx, 2, 1, (float32_t *)zHx_f32);
	arm_mat_init_f32(&KzHx, 4, 1, (float32_t *)KzHx_f32);
	arm_mat_init_f32(&KH, 4, 4, (float32_t *)KH_f32);
	arm_mat_init_f32(&IKH, 4, 4, (float32_t *)IKH_f32);


	// I = eye(4, 4);
	float32_t t_I[] = {  1, 0, 0, 0,
					 	 0, 1, 0, 0,
						 0, 0, 1, 0,
						 0, 0, 0, 1 };
	arm_mat_init_f32(&I, 4, 4, (float32_t *)t_I);


	//	HT = H.transpose()
	arm_status status = arm_mat_trans_f32(&this->H, &HT);


	// S = H @ P @ H.T + R
	status = arm_mat_mult_f32(&this->H, &this->P, &HP);
	status = arm_mat_mult_f32(&HP, &HT, &HPHT);
	status = arm_mat_add_f32(&HPHT, &this->R, &S);


	// K = P @ H.T @ inv(S)
	status = arm_mat_inverse_f32(&S, &S);
	status = arm_mat_mult_f32(&this->P, &HT, &PHT);
	status = arm_mat_mult_f32(&PHT, &S, &K);

	// x = x + K @ (z - H @ x)
	arm_mat_init_f32(&z, 2, 1, (float32_t *) (&t_z[0]) );
	arm_mat_mult_f32(&this->H, &this->x, &Hx);
	status = arm_mat_sub_f32(&z, &Hx, &zHx);
	status = arm_mat_mult_f32(&K, &zHx, &KzHx);
	status = arm_mat_add_f32(&this->x, &KzHx, &this->x);


	// P = (I - (K @ H)) @ P
	status = arm_mat_mult_f32(&K, &this->H, &KH);
	status = arm_mat_sub_f32(&I, &KH, &IKH);
	status = arm_mat_mult_f32(&IKH, &this->P, &this->P);

}


shared_ptr<vector<double>> KFTracking::getX() {

	shared_ptr<vector<double>> result = make_shared<vector<double>>();
	result->push_back(x.pData[0]);
	result->push_back(x.pData[1]);
	result->push_back(x.pData[2]);
	result->push_back(x.pData[3]);
	return result;
}

#else

shared_ptr<std::vector<double>> KFTracking::getX() {

	shared_ptr<vector<double>> result = make_shared<vector<double>>();
	result->push_back(x[0]);
	result->push_back(x[1]);
	return result;
}

void KFTracking::initialize(const vector<double> &params) {

	double init_x 	= params.at(0);
	double init_y 	= params.at(1);
	double std_x  	= params.at(2);
	double std_y  	= params.at(3);
	double dt 	 		= params.at(4);
	double process_noise = params.at(5);

	x.resize( 4 * 1 );
	H.resize( 2 * 4 );
	Q.resize( 4 * 4 );
	R.resize( 2 * 2 );
	P.resize( 4 * 4 );
	I.resize( 4 * 4 );
	A.resize( 4 * 4 );

	this->dt = dt;

	//	x = [init_x, init_y, 0, 0]
	double t_x[] = { init_x, init_y, 0, 0 };
	std::copy(std::begin(t_x), std::end(t_x), std::begin(this->x));


	double t_A[] = { 1, 0, this->dt, 	0,
									 0, 1, 0, 		 		this->dt,
									 0, 0, 1, 		 		0,
									 0, 0, 0,		 			1 };
	std::copy(std::begin(t_A), std::end(t_A), std::begin(this->A));


	double t_H[] = {  1, 0, 0, 0,
										0, 1, 0, 0 };
	std::copy(std::begin(t_H), std::end(t_H), std::begin(this->H));


	//	Q =	  [(dt**4)/4, 		0, 				(dt**3)/2, 		 0],
	//		  	[0, 				(dt**4)/4, 		 0, 			(dt**3)/2],
	//	      [(dt**3)/2, 		0, 				 dt**2, 		 0],
	//	      [0, 				(dt**3)/2, 		 0, 	 		 dt**2]]) * std_acc**2
	double t_Q[] = {  std::pow(this->dt, 4)/4, 	0,							std::pow(this->dt, 3)/2,	0,
					 	 0,							std::pow(this->dt, 4)/4,	0,							std::pow(this->dt, 3)/2,
						 std::pow(this->dt, 3)/2,	0,							std::pow(this->dt, 2),		0,
						 0,							std::pow(this->dt, 3)/2, 	0, 							std::pow(this->dt, 2) 	};
	std::copy(std::begin(t_Q), std::end(t_Q), std::begin(this->Q));
	scale(Q, std::pow(process_noise, 2), 4, 4);
	// arm_mat_scale_f32(&this->Q, std::pow(process_noise, 2), &this->Q);


	//	R = [x_std_meas**2,		0],
	//	    [0, 				y_std_meas**2]])
	double t_R[] = { std::pow(std_x, 2),  	0,
					 0,						std::pow(std_y, 2)	};
	std::copy(std::begin(t_R), std::end(t_R), std::begin(this->R));
//	arm_mat_init_f32(&this->R, 2, 2, (float32_t *)this->R_f32);


	//	P = eye(4, 4);
	double t_P[] = { 1, 0, 0, 0,
					 0, 1, 0, 0,
					 0, 0, 1, 0,
					 0, 0, 0, 1 };
	std::copy(std::begin(t_P), std::end(t_P), std::begin(this->P));

	// I = eye(4, 4);
	double t_I[] = { 1, 0, 0, 0,
					 0, 1, 0, 0,
					 0, 0, 1, 0,
					 0, 0, 0, 1 };
	std::copy(std::begin(t_I), std::end(t_I), std::begin(this->I));

}


void KFTracking::predict() {

	// x = A @ x
	vector<double> C_x( 4 * 1 );
	std::copy(std::begin(x), std::end(x), std::begin(C_x));
	mul(A, C_x, x, 4, 4, 1);


	// P = A @ P @ A.T + Q
	vector<double> AT( 4 * 4 );
	vector<double> AP( 4 * 4 );
	vector<double> APAT( 4 * 4 );


	// AT = A.transpose()
	std::copy(std::begin(A), std::end(A), std::begin(AT));
	tran(AT, 4, 4);

	mul(A, P, AP, 4, 4, 4);
	mul(AP, AT, APAT, 4, 4, 4);
	add(APAT, Q, P, 4, 4);

}


void KFTracking::update(const vector<double> &t_z) {

	vector<double> S( 2 * 2 );
	vector<double> HP( 2 * 4 );
	vector<double> HT( 4 * 2 );
	vector<double> HPHT( 2 * 2 );
	vector<double> PHT( 4 * 2 );
	vector<double> K( 4 * 2 );
	vector<double> Hx( 2 * 1 );
	vector<double> zHx( 2 * 1 );
	vector<double> KzHx( 4 * 1 );
	vector<double> KH( 4 * 4 );
	vector<double> IKH( 4 * 4 );


	//	HT = H.transpose()
	std::copy(std::begin(H), std::end(H), std::begin(HT));
	tran(HT, 2, 4);

	// S = H @ P @ H.T + R
	mul(H, P, HP, 2, 4, 4);
	mul(HP, HT, HPHT, 2, 4, 2);
	add(HPHT, R, S, 2, 2);

	// Inverse of S
	double a = S[0];
	double b = S[1];
	double c = S[2];
	double d = S[3];
	double det = (a * d) - (b * c);
	det = 1 / det;
	vector<double> SI = { d, -b, -c, a };
	scale(SI, det, 2, 2);

	// K = P @ H.T @ inv(S)
	mul(P, HT, PHT, 4, 4, 2);
	mul(PHT, SI, K, 4, 2, 2);


	// x = x + K @ (z - H @ x)
	vector<double> C_x( 4 * 1 );
	std::copy(std::begin(x), std::end(x), std::begin(C_x));
	vector<double> z = { t_z.at(0), t_z.at(1) };
	mul(H, x, Hx, 2, 4, 1);
	sub(z, Hx, zHx, 2, 1);
	mul(K, zHx, KzHx, 4, 2, 1);
	add(C_x, KzHx, x, 4, 1);

	// P = (I - (K @ H)) @ P
	vector<double> C_P( 4 * 4 );
	std::copy(std::begin(P), std::end(P), std::begin(C_P));
	mul(K, H, KH, 4, 2, 4);
	sub(I, KH, IKH, 4, 4);
	mul(IKH, C_P, P, 4, 4, 4);

}

#endif




