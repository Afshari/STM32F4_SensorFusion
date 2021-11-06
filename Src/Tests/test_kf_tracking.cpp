#include "tests/test_kf_tracking.h"

TestKFTracking::TestKFTracking() : TestBase() {

}


void TestKFTracking::init() {

	testInitialize();
	testPredict();
	testUpdate();

	printReport("Kalman Filter Tracking");
}


#ifdef USE_CMSIS_DSP

void TestKFTracking::testInitialize() {

	bool status = false;
	float matrixTolerance = 0.01;

	float arr_data[] = { 0, 0, 0.1, 0.1, 0.1, 1 };
	shared_ptr<vector<float>> data = make_shared<vector<float>>( vector<float>(std::begin(arr_data), std::end(arr_data)) );
	kf.initialize(data);

	arm_matrix_instance_f32 x;
	arm_matrix_instance_f32 A;
	arm_matrix_instance_f32 H;
	arm_matrix_instance_f32 Q;
	arm_matrix_instance_f32 R;
	arm_matrix_instance_f32 P;


	float32_t t_x[] = { 0, 0, 0, 0 };
	arm_mat_init_f32(&x, 4, 1, (float32_t *)t_x);
	status = checkEqual(kf.x, x, matrixTolerance, "There is Problem in KalmanFilter initialize() Function -- x Value");

	float32_t t_A[] = {  1, 0, 0.1, 	0,
					 	 0, 1, 0, 		0.1,
						 0, 0, 1, 		0,
						 0, 0, 0,		1 };
	arm_mat_init_f32(&A, 4, 4, (float32_t *)t_A);
	status = checkEqual(kf.A, A, matrixTolerance, "There is Problem in KalmanFilter initialize() Function -- A Value");


	float32_t t_H[] = {  1, 0, 0, 0,
					 	 0, 1, 0, 0 };
	arm_mat_init_f32(&H, 2, 4, (float32_t *)t_H);
	status = checkEqual(kf.H, H, matrixTolerance, "There is Problem in KalmanFilter initialize() Function -- H Value");


	float32_t t_Q[] = {  2.5e-05, 	0,			5.0e-04,	0,
					 	 0,			2.5e-05,	0, 			5.0e-04,
						 5.0e-04,	0,			1.0e-02,	0,
						 0,			5.0e-04, 	0, 			1.0e-02 	};
	arm_mat_init_f32(&Q, 4, 4, (float32_t *)t_Q);
	status = checkEqual(kf.Q, Q, 0.001, "There is Problem in KalmanFilter initialize() Function -- Q Value");


	float32_t t_R[] = {  0.01,  	0,
					 	 0,			0.01	};
	arm_mat_init_f32(&R, 2, 2, (float32_t *)t_R);
	status = checkEqual(kf.R, R, matrixTolerance, "There is Problem in KalmanFilter initialize() Function -- R Value");


	float32_t t_P[] = {  1, 0, 0, 0,
					 	 0, 1, 0, 0,
						 0, 0, 1, 0,
						 0, 0, 0, 1 };
	arm_mat_init_f32(&P, 4, 4, (float32_t *)t_P);
	status = checkEqual(kf.P, P, matrixTolerance, "There is Problem in KalmanFilter initialize() Function -- P Value");

}


void TestKFTracking::testPredict() {

	bool status = false;
	float matrixTolerance = 0.01;

	float arr_data[] = { 0, 0, 0.1, 0.1, 0.1, 1 };
	shared_ptr<vector<float>> data = make_shared<vector<float>>( vector<float>(std::begin(arr_data), std::end(arr_data)) );
	kf.initialize(data);
	kf.predict();

	arm_matrix_instance_f32 x;
	arm_matrix_instance_f32 P;


	float32_t t_x[] = { 0, 0, 0, 0 };
	arm_mat_init_f32(&x, 4, 1, (float32_t *)t_x);
	status = checkEqual(kf.x, x, matrixTolerance, "There is Problem in KalmanFilter predict() Function -- x Value");

	float32_t t_P[] = {  1.010025, 	0, 			0.1005, 	0,
					 	 0, 		1.010025, 	0, 			0.1005,
						 0.1005, 	0, 			1.01, 		0,
						 0, 		0.1005, 	0, 			1.01 };
	arm_mat_init_f32(&P, 4, 4, (float32_t *)t_P);
	status = checkEqual(kf.P, P, matrixTolerance, "There is Problem in KalmanFilter predict() Function -- P Value");


}


void TestKFTracking::testUpdate() {

	bool status = false;
	float matrixTolerance = 0.1;

	float arr_data[] = { 0, 0, 0.1, 0.1, 0.1, 1 };
	shared_ptr<vector<float>> data = make_shared<vector<float>>( vector<float>(std::begin(arr_data), std::end(arr_data)) );
	kf.initialize(data);
	kf.predict();

	float arr_z[] = { 107, 298 };
	vector<float> z( vector<float>(std::begin(arr_z), std::end(arr_z)) );
	kf.update(z);

	arm_matrix_instance_f32 x;
	arm_matrix_instance_f32 P;


	float32_t t_x[] = { 108.07, 300.98, 10.75, 29.94 };
	arm_mat_init_f32(&x, 4, 1, (float32_t *)t_x);
	status = checkEqual(kf.x, x, matrixTolerance, "There is Problem in KalmanFilter update() Function -- x Value");

	float32_t t_P[] = {	-0.0101255,		 0.,         	-0.00100751,  0.,
						 0.,         	-0.0101255, 	 0.,         -0.00100751,
						-0.00100751,  	 0.,          	 0.99989975,  0.,
						 0.,         	-0.00100751,  	 0.,          0.99989975 };
	arm_mat_init_f32(&P, 4, 4, (float32_t *)t_P);
	status = checkEqual(kf.P, P, 0.2, "There is Problem in KalmanFilter update() Function -- P Value");

}

#else

void TestKFTracking::testInitialize() {

	bool status = false;
	float matrixTolerance = 0.01;

	vector<double> data = { 0, 0, 0.1, 0.1, 0.1, 1 };
	kf.initialize(data);

	vector<double> x = { 0, 0, 0, 0 };
	status = checkEqual(kf.x, x, matrixTolerance, "There is Problem in KalmanFilter initialize() Function -- x Value");

	vector<double> A = {   1, 	0, 		0.1, 		0,
												 0, 	1, 		0, 			0.1,
												 0, 	0, 		1, 			0,
												 0, 	0, 		0,			1   };
	status = checkEqual(kf.A, A, matrixTolerance, "There is Problem in KalmanFilter initialize() Function -- A Value");


	vector<double> H = {  1, 0, 0, 0,
												0, 1, 0, 0 };
	status = checkEqual(kf.H, H, matrixTolerance, "There is Problem in KalmanFilter initialize() Function -- H Value");


	vector<double> Q = {   2.5e-05, 	0,				5.0e-04,	0,
												 0,					2.5e-05,	0, 				5.0e-04,
												 5.0e-04,		0,				1.0e-02,	0,
												 0,					5.0e-04, 	0, 				1.0e-02 	};
	status = checkEqual(kf.Q, Q, 0.001, "There is Problem in KalmanFilter initialize() Function -- Q Value");


	vector<double> R = {  0.01,  	0,
												0,			0.01	};
	status = checkEqual(kf.R, R, matrixTolerance, "There is Problem in KalmanFilter initialize() Function -- R Value");

	vector<double> P = {   1, 0, 0, 0,
												 0, 1, 0, 0,
												 0, 0, 1, 0,
												 0, 0, 0, 1 };
	status = checkEqual(kf.P, P, matrixTolerance, "There is Problem in KalmanFilter initialize() Function -- P Value");
}

void TestKFTracking::testPredict() {

	bool status = false;
	float matrixTolerance = 0.01;

	std::vector<double> data = { 0, 0, 0.1, 0.1, 0.1, 1 };
	kf.initialize(data);
	kf.predict();

	vector<double> x = { 0, 0, 0, 0 };
	status = checkEqual(kf.x, x, matrixTolerance, "There is Problem in KalmanFilter predict() Function -- x Value");

	vector<double> P = {   	1.010025, 		0, 					0.1005, 	0,
													0, 						1.010025, 	0, 				0.1005,
													0.1005, 			0, 					1.01, 		0,
													0, 						0.1005, 		0, 				1.01 };
	status = checkEqual(kf.P, P, matrixTolerance, "There is Problem in KalmanFilter predict() Function -- P Value");

}

void TestKFTracking::testUpdate() {

	bool status = false;
	float matrixTolerance = 0.1;

	vector<double> data = { 0, 0, 0.1, 0.1, 0.1, 1 };
	kf.initialize(data);
	kf.predict();

	vector<double> z = { 107, 298 };
	kf.update(z);

	vector<double> x = { 105.90, 295.01, 10.50, 29.3 };
	status = checkEqual(kf.x, x, matrixTolerance, "There is Problem in KalmanFilter update() Function -- x Value");

	vector<double> P = {	-0.0101255,		 	 0.,         		 -0.00100751,  		0.,
												 0.,         	  -0.0101255, 	 		0.,         	 -0.00100751,
												-0.00100751,  	 0.,          		0.99989975,  		0.,
												 0.,         		-0.00100751,  	 	0.,          		0.99989975 };
	status = checkEqual(kf.P, P, 0.2, "There is Problem in KalmanFilter update() Function -- P Value");
}


#endif







