

#include "kf_passive_suspension.h"

KFPassiveSuspension::KFPassiveSuspension() {

}



#ifdef USE_CMSIS_DSP

shared_ptr<vector<double>> KFPassiveSuspension::getX() {

	shared_ptr<vector<double>> result = make_shared<vector<double>>();
	result->push_back(x.pData[0]);
	result->push_back(x.pData[1]);
	result->push_back(x.pData[2]);
	result->push_back(x.pData[3]);
	return result;
}

void KFPassiveSuspension::initialize() {

	float32_t t_x[] = { 0, 0, 0, 0 };
	std::copy(std::begin(t_x), std::end(t_x), std::begin(this->x_f32));
	arm_mat_init_f32(&this->x, 4, 1, (float32_t *)this->x_f32);

	float32_t t_H[] = { 1, 0, 0, 0 };
	std::copy(std::begin(t_H), std::end(t_H), std::begin(this->H_f32));
	arm_mat_init_f32(&this->H, 1, 4, (float32_t *)this->H_f32);

	float32_t t_R[] = { 1e-6 };
	std::copy(std::begin(t_R), std::end(t_R), std::begin(this->R_f32));
	arm_mat_init_f32(&this->R, 1, 1, (float32_t *)this->R_f32);


    float32_t t_P[] = {
    		 1,     	0,      0,      0,
             0,     	1,      0,      0,
             0,     	0,      1,      0,
             0,     	0,      0,      1 };
	std::copy(std::begin(t_P), std::end(t_P), std::begin(this->P_f32));
	arm_mat_init_f32(&this->P, 4, 4, (float32_t *)this->P_f32);


	float32_t t_F[] = {
		   0.9998,    0.0004,    0.0010,   -0.0010,
		   0.0002,    0.9996,    0.0000,    0.0010,
		  -0.0437,   -0.0005,    0.9989,    0.0011,
		   0.3739,   -0.8856,    0.0098,    0.9897 	};
	std::copy(std::begin(t_F), std::end(t_F), std::begin(this->F_f32));
	arm_mat_init_f32(&this->F, 4, 4, (float32_t *)this->F_f32);


	float32_t t_Q[] = {
		   0.0000,    0.0000,   -0.0000,   -0.0000,
		   0.0000,    0.2233,   -0.0001,   -0.1276,
		  -0.0000,   -0.0001,    0.0000,    0.0000,
		  -0.0000,   -0.1276,    0.0000,    0.0875 	};
	std::copy(std::begin(t_Q), std::end(t_Q), std::begin(this->Q_f32));
	arm_mat_init_f32(&this->Q, 4, 4, (float32_t *)this->Q_f32);

}



void KFPassiveSuspension::predict() {

    // x = F @ x
	arm_status status = arm_mat_mult_f32(&this->F, &this->x, &this->x);

    // P = F @ P @ F.T + Q
	arm_matrix_instance_f32 FT;
	arm_matrix_instance_f32 FP;
	arm_matrix_instance_f32 FPFT;

	float FT_f32[4*4];
	float FP_f32[4*4];
	float FPFT_f32[4*4];

	std::copy(std::begin(this->init_44), std::end(this->init_44), std::begin(FT_f32));
	std::copy(std::begin(this->init_44), std::end(this->init_44), std::begin(FP_f32));
	std::copy(std::begin(this->init_44), std::end(this->init_44), std::begin(FPFT_f32));

	arm_mat_init_f32(&FT, 4, 4, (float32_t *)FT_f32);
	arm_mat_init_f32(&FP, 4, 4, (float32_t *)FP_f32);
	arm_mat_init_f32(&FPFT, 4, 4, (float32_t *)FPFT_f32);

	// FT = F.transpose()
	status = arm_mat_trans_f32(&this->F, &FT);

	status = arm_mat_mult_f32(&this->F, &this->P, &FP);
	status = arm_mat_mult_f32(&FP, &FT, &FPFT);
	status = arm_mat_add_f32(&FPFT, &Q, &this->P);

}


void KFPassiveSuspension::update(const double &param) {


	arm_matrix_instance_f32 S;
	arm_matrix_instance_f32 SI;
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


	float S_f32[1*1];
	float HP_f32[1*4];
	float HT_f32[4*1];
	float HPHT_f32[1*1];
	float PHT_f32[4*1];
	float K_f32[4*1];
	float Hx_f32[1*1];
	float zHx_f32[1*1];
	float KzHx_f32[4*1];
	float KH_f32[4*4];
	float IKH_f32[4*4];


	std::copy(std::begin(this->init_11), std::end(this->init_11), std::begin(S_f32));
	std::copy(std::begin(this->init_41), std::end(this->init_41), std::begin(HP_f32));
	std::copy(std::begin(this->init_41), std::end(this->init_41), std::begin(HT_f32));
	std::copy(std::begin(this->init_11), std::end(this->init_11), std::begin(HPHT_f32));
	std::copy(std::begin(this->init_41), std::end(this->init_41), std::begin(PHT_f32));
	std::copy(std::begin(this->init_41), std::end(this->init_41), std::begin(K_f32));
	std::copy(std::begin(this->init_11), std::end(this->init_11), std::begin(Hx_f32));
	std::copy(std::begin(this->init_11), std::end(this->init_11), std::begin(zHx_f32));
	std::copy(std::begin(this->init_41), std::end(this->init_41), std::begin(KzHx_f32));
	std::copy(std::begin(this->init_44), std::end(this->init_44), std::begin(KH_f32));
	std::copy(std::begin(this->init_44), std::end(this->init_44), std::begin(IKH_f32));



	arm_mat_init_f32(&S, 1, 1, (float32_t *)S_f32);
	arm_mat_init_f32(&HP, 1, 4, (float32_t *)HP_f32);
	arm_mat_init_f32(&HT, 4, 1, (float32_t *)HT_f32);
	arm_mat_init_f32(&HPHT, 1, 1, (float32_t *)HPHT_f32);
	arm_mat_init_f32(&PHT, 4, 1, (float32_t *)PHT_f32);
	arm_mat_init_f32(&K, 4, 1, (float32_t *)K_f32);
	arm_mat_init_f32(&Hx, 1, 1, (float32_t *)Hx_f32);
	arm_mat_init_f32(&zHx, 1, 1, (float32_t *)zHx_f32);
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
	float SI_f32[1*1] = { 1/S.pData[0] };
	arm_mat_init_f32(&SI, 1, 1, (float32_t *)SI_f32);
	status = arm_mat_mult_f32(&this->P, &HT, &PHT);
	status = arm_mat_mult_f32(&PHT, &SI, &K);

	// x = x + K @ (z - H @ x)
	float t_z[] = { (float)param };
	arm_mat_init_f32(&z, 1, 1, t_z );
	status = arm_mat_mult_f32(&this->H, &this->x, &Hx);
	status = arm_mat_sub_f32(&z, &Hx, &zHx);
	status = arm_mat_mult_f32(&K, &zHx, &KzHx);
	status = arm_mat_add_f32(&this->x, &KzHx, &this->x);


	// P = (I - (K @ H)) @ P
	status = arm_mat_mult_f32(&K, &this->H, &KH);
	status = arm_mat_sub_f32(&I, &KH, &IKH);
	status = arm_mat_mult_f32(&IKH, &this->P, &this->P);

}

#else

shared_ptr<vector<double>> KFPassiveSuspension::getX() {

	shared_ptr<vector<double>> result = make_shared<vector<double>>();
	result->push_back(x[0]);
	result->push_back(x[1]);
	result->push_back(x[2]);
	result->push_back(x[3]);
	return result;
}

void KFPassiveSuspension::initialize() {

	x.resize( 4 * 1 );
	P.resize( 4 * 4 );
	F.resize( 4 * 4 );
	H.resize( 1 * 4 );
	Q.resize( 4 * 4 );
	R.resize( 1 * 1 );
	HT.resize( 4 * 1 );
	_I.resize( 4 * 4 );

	double t_x[] = { 0, 0, 0, 0 };
	std::copy(std::begin(t_x), std::end(t_x), std::begin(this->x));

	double t_H[] = { 1, 0, 0, 0 };
	std::copy(std::begin(t_H), std::end(t_H), std::begin(this->H));
	std::copy(std::begin(t_H), std::end(t_H), std::begin(this->HT));

	double t_R[] = { 1e-6 };
	std::copy(std::begin(t_R), std::end(t_R), std::begin(this->R));

	double t_P[] = {
						 1,     	0,      0,      0,
             0,     	1,      0,      0,
             0,     	0,      1,      0,
             0,     	0,      0,      1 };
	std::copy(std::begin(t_P), std::end(t_P), std::begin(this->P));


	double t_F[] = {
		   0.9998,    0.0004,    0.0010,   -0.0010,
		   0.0002,    0.9996,    0.0000,    0.0010,
		  -0.0437,   -0.0005,    0.9989,    0.0011,
		   0.3739,   -0.8856,    0.0098,    0.9897 	};
	std::copy(std::begin(t_F), std::end(t_F), std::begin(this->F));


	double t_Q[] = {
		   0.0000,    0.0000,   -0.0000,   -0.0000,
		   0.0000,    0.2233,   -0.0001,   -0.1276,
		  -0.0000,   -0.0001,    0.0000,    0.0000,
		  -0.0000,   -0.1276,    0.0000,    0.0875 	};
	std::copy(std::begin(t_Q), std::end(t_Q), std::begin(this->Q));
}

void KFPassiveSuspension::predict() {

	vector<double> C_x( 4 * 1 );
	std::copy(std::begin(x), std::end(x), std::begin(C_x));

	// x = F @ x
	mul(F, C_x, x, 4, 4, 1);

  // P = F @ P @ F.T + Q
	vector<double> FT( 4 * 4 );
	vector<double> FP( 4 * 4 );
	vector<double> FPFT( 4 * 4 );

	// FT = F.transpose()
	std::copy(std::begin(F), std::end(F), std::begin(FT));
	tran(FT, 4, 4);

	mul(F, P, FP, 4, 4, 4);
	mul(FP, FT, FPFT, 4, 4, 4);
	add(FPFT, Q, P, 4, 4);
}

void KFPassiveSuspension::update(const double &param) {

	// I = eye(4, 4);
	vector<double> I = {   
									 1, 0, 0, 0,
									 0, 1, 0, 0,
									 0, 0, 1, 0,
									 0, 0, 0, 1 };

	vector<double> S( 1 * 1 );
	vector<double> HP( 1 * 4 );
	vector<double> HPHT( 1 * 1 );
	vector<double> PHT( 4 * 1 );
	vector<double> K( 4 * 1 );
	vector<double> Hx( 1 * 1 );
	vector<double> zHx( 1 * 1 );
	vector<double> KzHx( 4 * 1 );
	vector<double> KH( 4 * 4 );
	vector<double> IKH( 4 * 4 );


	// S = H @ P @ H.T + R
	mul(H, P, HP, 1, 4, 4);
	mul(HP, HT, HPHT, 1, 4, 1);
	add(HPHT, R, S, 1, 1);

	// K = P @ H.T @ inv(S)
	vector<double> SI = { 1/S[0] };
	mul(P, HT, PHT, 4, 4, 1);
	mul(PHT, SI, K, 4, 1, 1);

	// DebugSWV::print_debug("S: %.7f and SI: %.2f\r\n", S[0], SI[0]);

	vector<double> C_x( 4 * 1 );
	std::copy(std::begin(x), std::end(x), std::begin(C_x));
	// x = x + K @ (z - H @ x)
	vector<double> z = { param };
	mul(H, x, Hx, 1, 4, 1);
	sub(z, Hx, zHx, 1, 1);
	mul(K, zHx, KzHx, 4, 1, 1);
	add(C_x, KzHx, x, 4, 1);

	vector<double> C_P( 4 * 4 );
	std::copy(std::begin(P), std::end(P), std::begin(C_P));

	// P = (I - (K @ H)) @ P
	mul(K, H, KH, 4, 1, 4);
	sub(I, KH, IKH, 4, 4);
	mul(IKH, C_P, P, 4, 4, 4);

}



#endif






