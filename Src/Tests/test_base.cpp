
#include "Tests/test_base.h"


TestBase::TestBase() {

	printf(" >>>>> Initializing Unit Test <<<<<\r\n");

	failure = 0;
	success = 0;

}

void TestBase::printReport(const char *test_name) {

	printf("Class -> %s -- Total Number of Test: %d -- Success: %d -- Failure: %d\r\n",
			test_name, this->failure + this->success, this->success, this->failure);
	printf("---------------------\r\n");
}



//void TestBase::print_debug(const char *fmt, ...) {

//    char buffer[100];

//    va_list args;
//    va_start(args, fmt);
//    int rc = vsnprintf(buffer, sizeof(buffer), fmt, args);
//    va_end(args);

//    strcat(buffer, "\r\n");

//		write_swv(buffer, rc+2);
//}

//int TestBase::write_swv(char *ptr, int len) {

//	int DataIdx;

//	for (DataIdx = 0; DataIdx < len; DataIdx++) {
//		ITM_SendChar(*ptr);
//		ptr += 1;
//	}

//	return len;
//}


int TestBase::getFailure() {
	return failure;
}

int TestBase::getSuccess() {
	return success;
}

bool TestBase::checkEqualBool(bool value, string msg) {

	if(value == true) {
		success += 1;
		return true;
	} else {
		failure += 1;
		printf("FAILURE: %s -> Current Value: %d -- Expected: true\r\n", msg.c_str(), value);
		return false;
	}

}

bool TestBase::checkEqual(int value1, int value2, std::string msg) {

	if(value1 == value2) {
		success += 1;
		return true;
	} else {
		failure += 1;
		printf("FAILURE: %s -> Current Value: %d -- Expected: %d\r\n", msg.c_str(), value1, value2);
		return false;
	}
}

bool TestBase::checkEqual(bool value1, bool value2, string msg) {

	if(value1 == value2) {
		success += 1;
		return true;
	} else {
		failure += 1;
		printf("FAILURE: %s -> Current Value: %s -- Expected: %s\r\n", msg.c_str(),
				(value1 == true) ? "true" : "false", (value2 == true) ? "true" : "false");
		return false;
	}
}

bool TestBase::checkEqual(string value1, string value2, string msg) {

	if(value1 == value2) {
		success += 1;
		return true;
	} else {
		failure += 1;
		printf("FAILURE: %s -> Current Value: %s -- Expected: %s\r\n", msg.c_str(), value1.c_str(), value2.c_str());
		return false;
	}
}

bool TestBase::checkEqual(float value1, float value2, float tol, string msg) {

	if( abs(value1 - value2) < tol ) {
		success += 1;
		return true;
	} else {
		failure += 1;
		printf("FAILURE: %s -> Current Value: %f -- Expected: %f\r\n", msg.c_str(), value1, value2);
		return false;
	}
}


bool TestBase::checkEqual(double *value1, double *value2, int numRows, int numCols, float tolerance, string msg) {

	bool isEqual = true;


	for(int i = 0; i < numRows * numCols; i++) {

		if( abs(value1[i] - value2[i]) > tolerance ) {
			isEqual = false;
			break;
		}
	}

	if(isEqual) success += 1;
	else {
		failure += 1;
		printf("FAILURE: %s\r\n", msg.c_str());
	}


	return isEqual;
}


bool TestBase::checkEqual(const vector<double> &value1, const vector<double> &value2, float tolerance, string msg) {

	bool isEqual = true;


	for(int i = 0; i < value1.size(); i++) {

		if( abs(value1.at(i) - value2.at(i)) > tolerance ) {
			isEqual = false;
			break;
		}
	}

	if(isEqual) success += 1;
	else {
		failure += 1;
		printf("FAILURE: %s\r\n", msg.c_str());
	}


	return isEqual;
}

bool TestBase::checkEqual(const Matrix &value1, const Matrix &value2, float tolerance, string msg) {
	
	bool isEqual = true;

	if(value1.columnSize() != value2.columnSize() || value1.rowSize() != value2.rowSize())
		isEqual = false;

	for(int i = 0; i < value1.rowSize(); i++) {
			for(int j = 0; j < value1.columnSize(); j++) {
				if( std::abs( value1.at(i, j) - value2.at(i, j) ) > tolerance ) {
					isEqual = false;
					break;
				}
			}
	}

	if(isEqual) success += 1;
	else {
		failure += 1;
		printf("FAILURE: %s\r\n", msg.c_str());
	}


	return isEqual;
}

#ifdef USE_CMSIS_DSP
bool TestBase::checkEqualArmMatrix(arm_matrix_instance_f32 value1, arm_matrix_instance_f32 value2, float tolerance, std::string msg) {

	bool isEqual = true;

	if(value1.numCols != value2.numCols || value1.numRows != value2.numRows)
		isEqual = false;

	for(int i = 0; i < value1.numRows * value1.numCols; i++) {

		if( fabsf(value1.pData[i] - value2.pData[i]) > tolerance ) {
			isEqual = false;
			break;
		}
	}

	if(isEqual) success += 1;
	else {
		failure += 1;
		printf("FAILURE: %s\r\n", msg.c_str());
	}


	return isEqual;
}
#endif

