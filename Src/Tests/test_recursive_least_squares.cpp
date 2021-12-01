
#include "Tests/test_recursive_least_squares.h"

TestRecursiveLeastSquares::TestRecursiveLeastSquares() : TestBase() {

}



#ifdef USE_CMSIS_DSP

void TestRecursiveLeastSquares::init() {

	testInitialize();
//	testCalculate();

	printReport("Recursive Least Squares");
}

void TestRecursiveLeastSquares::testInitialize() {

	bool status = false;
	float matrixTolerance = 0.01;

	vector<double> data = { 8, 7, 0.1 };
	rls.initialize(data);

	arm_matrix_instance_f32 x;
	arm_matrix_instance_f32 R;
	arm_matrix_instance_f32 P;


	array<float32_t, 2> t_x = { 8, 7 };
	arm_mat_init_f32(&x, 2, 1, (float32_t *)t_x.data());
	status = checkEqualArmMatrix(rls.x, x, matrixTolerance, "There is Problem in Recursive Least Squares initialize() Function -- x Value");


	float32_t t_R[] = { 0.1 };
	arm_mat_init_f32(&R, 1, 1, (float32_t *)t_R);
	status = checkEqualArmMatrix(rls.R, R, matrixTolerance, "There is Problem in Recursive Least Squares initialize() Function -- R Value");


	float32_t t_P[] = {  1, 0,
					 	 0, 1 };
	arm_mat_init_f32(&P, 2, 2, (float32_t *)t_P);
	status = checkEqualArmMatrix(rls.P, P, matrixTolerance, "There is Problem in Recursive Least Squares initialize() Function -- P Value");
}


void TestRecursiveLeastSquares::testCalculate() {

	bool status = false;
	float matrixTolerance = 0.01;

	vector<double> data = { 8, 7, 0.1 };
	rls.initialize(data);


	arm_matrix_instance_f32 x;
	arm_matrix_instance_f32 P;

	float32_t t_x[] = { 7.890, 6.970 };
	arm_mat_init_f32(&x, 2, 1, (float32_t *)t_x);
	rls.x = x;

	float32_t t_P[] = {  0.49766082, -0.49248969, -0.49248969,  0.50736237 };
	arm_mat_init_f32(&P, 2, 2, (float32_t *)t_P);
	rls.P = P;

	vector<double> parm = { 1, 0.932065, 14.6714 };
	rls.calculate(parm);
	
	// The Result is not very accurate, more details on it in Demonstration Video
	// The Algorithm with Custom LinearAlgebra functions are more accurate
}


#else

void TestRecursiveLeastSquares::init() {

	testInitialize();
	testCalculate();

	printReport("Recursive Least Squares");
}


void TestRecursiveLeastSquares::testInitialize() {

	bool status = false;
	float matrixTolerance = 0.01;

	double arr_data[] = { 8, 7, 0.1 };
	vector<double> data = { 8, 7, 0.1 };
	rls.initialize(data);

	Matrix x { 2, 1, { 8, 7 } };
	status = checkEqual(rls.x, x, matrixTolerance, "There is Problem in Recursive Least Squares initialize() Function -- x Value");

	Matrix R { 1, 1, { 0.1 } };
	status = checkEqual(rls.R, R, matrixTolerance, "There is Problem in Recursive Least Squares initialize() Function -- R Value");

	Matrix P { 2, 2, { 1, 0,
										 0, 1 } };
	status = checkEqual(rls.P, P, matrixTolerance, "There is Problem in Recursive Least Squares initialize() Function -- P Value");
}

void TestRecursiveLeastSquares::testCalculate() {

	Matrix ref_x { 2, 1, { 7.94839, 6.9164 } };
	
	bool status = false;
	float matrixTolerance = 0.01;

	vector<double> data = { 8, 7, 0.1 };
	rls.initialize(data);

	Matrix x { 2, 1, { 7.872127, 6.96022 } };
	Matrix P { 2, 2, { 	0.48108, 	-0.48654,
											-0.48654, 0.50502 } };
	
	rls.x = x;
	rls.P = P;

	vector<double> parm = { 1, 0.932065, 14.6714 };
	rls.calculate(parm);
	Matrix result_x = rls.getX();
	
	checkEqual(rls.x, ref_x, matrixTolerance, " ");
	
}

#endif

