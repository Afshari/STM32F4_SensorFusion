#ifndef _RECURSIVE_LEAST_SQUARES_H_
#define _RECURSIVE_LEAST_SQUARES_H_

#include <string>
#include <cmath>
#include <algorithm>
#include <memory>
#include <array>
#include <vector>
#include "app.h"

#ifndef GTEST
#include "stm32f4xx_hal.h"
#endif

#ifdef USE_CMSIS_DSP
#include "arm_math.h"
#endif

#include "matrix.h"

using std::string;
using std::array;
using std::vector;
using std::shared_ptr;
using std::make_shared;

class RecursiveLeastSquares {

public:
	RecursiveLeastSquares();

	virtual Matrix getX();
	virtual void initialize(const vector<double> &params);
	virtual void calculate (const vector<double> &params);

private:
	Matrix x { 1, 1 };
	Matrix R { 1, 1 };
	Matrix P { 1, 1 };

	friend class TestRecursiveLeastSquares;

};

#endif /* _RECURSIVE_LEAST_SQUARES_H_ */

