#include "Tests/test_kf_tracking.h"

TestKFTracking::TestKFTracking() : TestBase() {

}


void TestKFTracking::init() {

	testInitialize();
	testPredict();
	testUpdate();

	printReport("Kalman Filter Tracking");
}


void TestKFTracking::testInitialize() {

	bool status = false;
	float matrixTolerance = 0.01;

	vector<double> data = { 0, 0, 0.1, 0.1, 0.1, 1 };
	kf.initialize(data);

	Matrix x { 4, 1, { 0, 0, 0, 0 } };
	status = checkEqual(kf.x, x, matrixTolerance, "There is Problem in KalmanFilter initialize() Function -- x Value");

	Matrix A = { 4, 4, {   1, 	0, 		0.1, 		0,
												 0, 	1, 		0, 			0.1,
												 0, 	0, 		1, 			0,
												 0, 	0, 		0,			1   } };
	status = checkEqual(kf.A, A, matrixTolerance, "There is Problem in KalmanFilter initialize() Function -- A Value");


	Matrix H = { 2, 4, {  1, 0, 0, 0,
												0, 1, 0, 0 } };
	status = checkEqual(kf.H, H, matrixTolerance, "There is Problem in KalmanFilter initialize() Function -- H Value");


	Matrix Q = { 4, 4, {   2.5e-05, 	0,				5.0e-04,	0,
												 0,					2.5e-05,	0, 				5.0e-04,
												 5.0e-04,		0,				1.0e-02,	0,
												 0,					5.0e-04, 	0, 				1.0e-02 	} };
	status = checkEqual(kf.Q, Q, 0.001, "There is Problem in KalmanFilter initialize() Function -- Q Value");


	Matrix R = { 2, 2, {  0.01,  	0,
												0,			0.01	} };
	status = checkEqual(kf.R, R, matrixTolerance, "There is Problem in KalmanFilter initialize() Function -- R Value");

	Matrix P = { 4, 4, {   1, 0, 0, 0,
												 0, 1, 0, 0,
												 0, 0, 1, 0,
												 0, 0, 0, 1 } }; 
	status = checkEqual(kf.P, P, matrixTolerance, "There is Problem in KalmanFilter initialize() Function -- P Value");
}

void TestKFTracking::testPredict() {

	bool status = false;
	float matrixTolerance = 0.01;

	std::vector<double> data = { 0, 0, 0.1, 0.1, 0.1, 1 };
	kf.initialize(data);
	kf.predict();

	Matrix x = { 4, 1, { 0, 0, 0, 0 } };
	status = checkEqual(kf.x, x, matrixTolerance, "There is Problem in KalmanFilter predict() Function -- x Value");

	Matrix P = { 4, 4, {  	1.010025, 		0, 					0.1005, 	0,
													0, 						1.010025, 	0, 				0.1005,
													0.1005, 			0, 					1.01, 		0,
													0, 						0.1005, 		0, 				1.01 } };
	status = checkEqual(kf.P, P, matrixTolerance, "There is Problem in KalmanFilter predict() Function -- P Value");

}

void TestKFTracking::testUpdate() {

	bool status = false;
	float matrixTolerance = 0.1;

	vector<double> data = { 0, 0, 0.1, 0.1, 0.1, 1 };
	kf.initialize(data);
	kf.predict();

	//vector<double> z = { 107, 298 };
	Matrix z { 2, 1, { 107, 298 } };
	kf.update(z);

	Matrix x = { 4, 1, { 105.90, 295.01, 10.50, 29.3 } };
	status = checkEqual(kf.x, x, matrixTolerance, "There is Problem in KalmanFilter update() Function -- x Value");

	Matrix P = {	4, 4, { -0.0101255,		 	 0.,         		 -0.00100751,  		0.,
												 0.,         	  -0.0101255, 	 		0.,         	 -0.00100751,
												-0.00100751,  	 0.,          		0.99989975,  		0.,
												 0.,         		-0.00100751,  	 	0.,          		0.99989975 } };
	status = checkEqual(kf.P, P, 0.2, "There is Problem in KalmanFilter update() Function -- P Value");
}







