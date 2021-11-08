

#include "Tests/test_kf_passive_suspension.h"

TestKFPassiveSuspension::TestKFPassiveSuspension() {



}

void TestKFPassiveSuspension::init() {

	testInitialize();

	printReport("Kalman Filter Passive Suspension");
}


#ifdef USE_CMSIS_DSP

void TestKFPassiveSuspension::testInitialize() {

	bool status = false;
	float matrixTolerance = 0.01;
	KFPassiveSuspension kf_passive_suspension;
	kf_passive_suspension.initialize();


	float32_t t_x[] = { 0, 0, 0, 0 };
	arm_matrix_instance_f32 x;
	arm_mat_init_f32(&x, 4, 1, (float32_t *)t_x);

	float32_t t_H[] = { 1, 0, 0, 0 };
	arm_matrix_instance_f32 H;
	arm_mat_init_f32(&H, 1, 4, (float32_t *)t_H);

	float32_t t_R[] = { 1e-6 };
	arm_matrix_instance_f32 R;
	arm_mat_init_f32(&R, 1, 1, (float32_t *)t_R);


    float32_t t_P[] = {
    		 1,     	0,      0,      0,
             0,     	1,      0,      0,
             0,     	0,      1,      0,
             0,     	0,      0,      1 };
    arm_matrix_instance_f32 P;
	arm_mat_init_f32(&P, 4, 4, (float32_t *)t_P);


	float32_t t_F[] = {
		   0.9998,    0.0004,    0.0010,   -0.0010,
		   0.0002,    0.9996,    0.0000,    0.0010,
		  -0.0437,   -0.0005,    0.9989,    0.0011,
		   0.3739,   -0.8856,    0.0098,    0.9897 	};
	arm_matrix_instance_f32 F;
	arm_mat_init_f32(&F, 4, 4, (float32_t *)t_F);


	float32_t t_Q[] = {
		   0.0000,    0.0000,   -0.0000,   -0.0000,
		   0.0000,    0.2233,   -0.0001,   -0.1276,
		  -0.0000,   -0.0001,    0.0000,    0.0000,
		  -0.0000,   -0.1276,    0.0000,    0.0875 	};
	arm_matrix_instance_f32 Q;
	arm_mat_init_f32(&Q, 4, 4, (float32_t *)t_Q);

	status = checkEqualArmMatrix(kf_passive_suspension.x, x, matrixTolerance, "");
	status = checkEqualArmMatrix(kf_passive_suspension.H, H, matrixTolerance, "");
	status = checkEqualArmMatrix(kf_passive_suspension.R, R, matrixTolerance, "");
	status = checkEqualArmMatrix(kf_passive_suspension.P, P, matrixTolerance, "");
	status = checkEqualArmMatrix(kf_passive_suspension.F, F, matrixTolerance, "");
	status = checkEqualArmMatrix(kf_passive_suspension.Q, Q, matrixTolerance, "");

}

#else

void TestKFPassiveSuspension::testInitialize() {

	bool status = false;
	float matrixTolerance = 0.01;
	KFPassiveSuspension kf_passive_suspension;
	kf_passive_suspension.initialize();

	vector<double> x = { 0, 0, 0, 0 };


	vector<double> H = { 1, 0, 0, 0 };

	vector<double> R = { 1e-6 };

	vector<double> P = {
						 1,     	0,      0,      0,
             0,     	1,      0,      0,
             0,     	0,      1,      0,
             0,     	0,      0,      1 };


	vector<double> F = {
		   0.9998,    0.0004,    0.0010,   -0.0010,
		   0.0002,    0.9996,    0.0000,    0.0010,
		  -0.0437,   -0.0005,    0.9989,    0.0011,
		   0.3739,   -0.8856,    0.0098,    0.9897 	};


	vector<double> Q = {
		   0.0000,    0.0000,   -0.0000,   -0.0000,
		   0.0000,    0.2233,   -0.0001,   -0.1276,
		  -0.0000,   -0.0001,    0.0000,    0.0000,
		  -0.0000,   -0.1276,    0.0000,    0.0875 	};


	status = checkEqual(kf_passive_suspension.x, x, matrixTolerance, "");
	status = checkEqual(kf_passive_suspension.H, H, matrixTolerance, "");
	status = checkEqual(kf_passive_suspension.R, R, matrixTolerance, "");
	status = checkEqual(kf_passive_suspension.P, P, matrixTolerance, "");
	status = checkEqual(kf_passive_suspension.F, F, matrixTolerance, "");
	status = checkEqual(kf_passive_suspension.Q, Q, matrixTolerance, "");

}

#endif






