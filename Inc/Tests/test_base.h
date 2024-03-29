
#ifndef _TESTS_TEST_BASE_H_
#define _TESTS_TEST_BASE_H_

#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include <string>
#include <vector>

#include "stm32f4xx_hal.h"
#include "app.h"
#include "matrix.h"

#ifdef USE_CMSIS_DSP
#include "arm_math.h"
#endif

using std::vector;
using std::string;

class TestBase {

public:
	TestBase();
	int getFailure();
	int getSuccess();

protected:
	int failure;
	int success;

protected:
	bool checkEqual(int value1, int value2, string msg = "");
	bool checkEqualBool(bool value, string msg = "");
	bool checkEqual(bool value1, bool value2, string msg = "");
	bool checkEqual(string value1, string value2, string msg = "");
	bool checkEqual(float value1, float value2, float tol, string msg = "");
	bool checkEqual(double *value1, double *value2, int numRows, int numCols, float tolerance = 0, string msg = "");
	bool checkEqual(const vector<double> &value1, const vector<double> &value2, float tolerance = 0, string msg = "");
	bool checkEqual(const Matrix &value1, const Matrix &value2, float tolerance = 0, string msg = "");
	

#ifdef USE_CMSIS_DSP
	bool checkEqualArmMatrix(arm_matrix_instance_f32 value1, arm_matrix_instance_f32 value2, float tolerance, string msg = "");
#endif

	void printReport(const char *test_name);
	void print_debug(const char *fmt, ...);
	int write_swv(char *ptr, int len);

};



#endif /* _TESTS_TEST_BASE_H_ */