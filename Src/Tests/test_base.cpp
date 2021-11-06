
#include "Tests/test_base.h"


TestBase::TestBase() {

	print_debug(" >>>>> Initializing Unit Test <<<<< ");

	failure = 0;
	success = 0;

}

void TestBase::printReport(const char *test_name) {

	print_debug("Class -> %s -- Total Number of Test: %d -- Success: %d -- Failure: %d",
			test_name, this->failure + this->success, this->success, this->failure);
	print_debug("---------------------");
}



void TestBase::print_debug(const char *fmt, ...) {

    char buffer[100];

    va_list args;
    va_start(args, fmt);
    int rc = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    strcat(buffer, "\r\n");

		write_swv(buffer, rc+2);
}

int TestBase::write_swv(char *ptr, int len) {

	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++) {
		ITM_SendChar(*ptr);
		ptr += 1;
	}

	return len;
}


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
		printf("FAILURE: %s -> Current Value: %d -- Expected: %d", msg.c_str(), value1, value2);
		return false;
	}
}

bool TestBase::checkEqual(bool value1, bool value2, string msg) {

	if(value1 == value2) {
		success += 1;
		return true;
	} else {
		failure += 1;
		printf("FAILURE: %s -> Current Value: %s -- Expected: %s", msg.c_str(),
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
		printf("FAILURE: %s -> Current Value: %s -- Expected: %s", msg.c_str(), value1.c_str(), value2.c_str());
		return false;
	}
}

bool TestBase::checkEqual(float value1, float value2, float tol, string msg) {

	if( abs(value1 - value2) < tol ) {
		success += 1;
		return true;
	} else {
		failure += 1;
		printf("FAILURE: %s -> Current Value: %f -- Expected: %f", msg.c_str(), value1, value2);
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
		printf("FAILURE: %s", msg.c_str());
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
		printf("FAILURE: %s", msg.c_str());
	}


	return isEqual;
}





