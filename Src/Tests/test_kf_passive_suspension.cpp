

#include "Tests/test_kf_passive_suspension.h"

TestKFPassiveSuspension::TestKFPassiveSuspension() {

}

void TestKFPassiveSuspension::init() {

	testInitialize();
	testPredict();
	testUpdate_1();
	testUpdate_2();

	printReport("Kalman Filter Passive Suspension");
}

void TestKFPassiveSuspension::testInitialize() {

	bool status = false;
	float matrixTolerance = 0.01;
	KFPassiveSuspension kf_passive_suspension;
	kf_passive_suspension.initialize();

	Matrix x { 4, 1, { 0, 0, 0, 0 } };

	Matrix H { 1, 4, { 1, 0, 0, 0 } };

	Matrix R { 1, 1, { 1e-6 } };

	Matrix P { 4, 4, {
									 1,     	0,      0,      0,
									 0,     	1,      0,      0,
									 0,     	0,      1,      0,
									 0,     	0,      0,      1 } };

	Matrix F { 4, 4, {
									 0.9998,    0.0004,    0.0010,   -0.0010,
									 0.0002,    0.9996,    0.0000,    0.0010,
									-0.0437,   -0.0005,    0.9989,    0.0011,
									 0.3739,   -0.8856,    0.0098,    0.9897 	} };

	Matrix Q { 4, 4, {
									 0.0000,    0.0000,   -0.0000,   -0.0000,
									 0.0000,    0.2233,   -0.0001,   -0.1276,
									-0.0000,   -0.0001,    0.0000,    0.0000,
									-0.0000,   -0.1276,    0.0000,    0.0875 	} };

	status = checkEqual(kf_passive_suspension.x, x, matrixTolerance, "");
	status = checkEqual(kf_passive_suspension.H, H, matrixTolerance, "");
	status = checkEqual(kf_passive_suspension.R, R, matrixTolerance, "");
	status = checkEqual(kf_passive_suspension.P, P, matrixTolerance, "");
	status = checkEqual(kf_passive_suspension.F, F, matrixTolerance, "");
	status = checkEqual(kf_passive_suspension.Q, Q, matrixTolerance, "");

}

void TestKFPassiveSuspension::testPredict() {
	
	Matrix ref_x { 4, 1, { -19.9209118,   -8.63996359,  57.37623886, 106.7661835 } };
	
	Matrix x { 4, 1, { -19.87092819,  -8.7468581,   56.44568757, 107.00571172 } };
	Matrix P { 4, 4, { 
					 1.20123548e-07,  3.22008851e-07, -1.74187938e-07, -7.85690194e-06,
           3.22008851e-07,  4.87621894e-06, -3.47648886e-07, -5.13354639e-05,
          -1.74187938e-07, -3.47648886e-07,  1.53978360e-06,  9.70202154e-06,
          -7.85690194e-06, -5.13354639e-05,  9.70202154e-06,  8.70548962e-04 } };
	
	KFPassiveSuspension kf_passive_suspension;
	kf_passive_suspension.initialize();
	kf_passive_suspension.x = x;
	kf_passive_suspension.P = P;
					
	kf_passive_suspension.predict();
	
	Matrix result_x = kf_passive_suspension.getX();
					
	checkEqual(result_x, ref_x, 0.1, "");
}

void TestKFPassiveSuspension::testUpdate_1() {

	Matrix ref_x { 4, 1, { -26.35912618, -12.34415419, 130.12046332, 114.68371551 } };
	
	Matrix x { 4, 1, { -26.31506731, -12.31076066, 130.04864341, 113.147455755 } };
	Matrix P { 4, 4, { 
						6.87481157e-08,  5.21062438e-08, -1.12065612e-07, -2.39713295e-06,
            5.21062438e-08,  3.92864018e-07, -3.97152699e-08, -5.76981182e-06,
           -1.12065612e-07, -3.97152699e-08,  1.42395914e-06,  3.32492513e-06,
           -2.39713295e-06, -5.76981182e-06,  3.32492513e-06,  1.61811052e-04 } };

	KFPassiveSuspension kf_passive_suspension;
	kf_passive_suspension.initialize();
	kf_passive_suspension.x = x;
	kf_passive_suspension.P = P;
					 
	double z = -27;
	kf_passive_suspension.update( z );
					 
	Matrix result_x = kf_passive_suspension.getX(); 
					 
	checkEqual(result_x, ref_x, 0.1, "");
}


void TestKFPassiveSuspension::testUpdate_2() {

	Matrix ref_x { 4, 1, { -32.3227836,  -10.67079103, -78.69531955, -51.21634435 } };
	
	Matrix x { 4, 1, { -32.27614212, -10.63519708, -78.76173833, -52.83574217 } };
	Matrix P { 4, 4, { 
						6.88723349e-08,  5.25591970e-08, -9.80761578e-08, -2.39125606e-06,
            5.25591970e-08,  3.94515669e-07,  1.12959353e-08, -5.74838229e-06,
           -9.80761578e-08,  1.12959353e-08,  2.99943945e-06,  3.98677601e-06,
           -2.39125606e-06, -5.74838229e-06,  3.98677601e-06,  1.62089092e-04 } };

	KFPassiveSuspension kf_passive_suspension;
	kf_passive_suspension.initialize();
	kf_passive_suspension.x = x;
	kf_passive_suspension.P = P;
					 
	double z = -33;
	kf_passive_suspension.update( z );
					 
	Matrix result_x = kf_passive_suspension.getX(); 
					 
	checkEqual(result_x, ref_x, 0.1, "");
}







