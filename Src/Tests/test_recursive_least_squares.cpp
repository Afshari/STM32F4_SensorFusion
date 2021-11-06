
#include "Tests/test_recursive_least_squares.h"

TestRecursiveLeastSquares::TestRecursiveLeastSquares() : TestBase() {

}



#ifdef USE_CMSIS_DSP

void TestRecursiveLeastSquares::init() {

	testInitialize();
	testCalculate();

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


	float32_t t_x[] = { 8, 7 };
	arm_mat_init_f32(&x, 2, 1, (float32_t *)t_x);
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


	vector<double> x = { 8, 7 };
	status = checkEqual(rls.x, x, matrixTolerance, "There is Problem in Recursive Least Squares initialize() Function -- x Value");



	vector<double> R = { 0.1 };
	status = checkEqual(rls.R, R, matrixTolerance, "There is Problem in Recursive Least Squares initialize() Function -- R Value");


	vector<double> P = {  1, 0,
												0, 1 };
	status = checkEqual(rls.P, P, matrixTolerance, "There is Problem in Recursive Least Squares initialize() Function -- P Value");
}

void TestRecursiveLeastSquares::testCalculate() {

	bool status = false;
	float matrixTolerance = 0.01;

	vector<double> data = { 8, 7, 0.1 };
	rls.initialize(data);

	double x[] = { 7.890, 6.970 };
	std::copy(std::begin(x), std::end(x), std::begin(rls.x));

	double P[] = {  0.49766082, -0.49248969, -0.49248969,  0.50736237 };
	std::copy(std::begin(P), std::end(P), std::begin(rls.P));


	float arr_parm[] = { 1, 0.932065, 14.6714 };
	vector<double> parm = { 1, 0.932065, 14.6714 };
	rls.calculate(parm);
}

#endif

