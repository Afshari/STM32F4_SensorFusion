#ifndef _KF_TRACKING_H_
#define _KF_TRACKING_H_

#include <string>
#include <cmath>
#include <algorithm>
#include <memory>
#include <vector>
#include <cmath>
#include "app.h"

#ifndef GTEST
#include "stm32f4xx_hal.h"
#endif

#ifdef USE_CMSIS_DSP
#include "arm_math.h"
#endif

#include "matrix.h"

using std::string;
using std::shared_ptr;
using std::make_shared;
using std::vector;

class KFTracking {

public:
	KFTracking();

	virtual void initialize(const vector<double> &params);
	virtual void predict();
	virtual void update(const vector<double> &z);
	virtual Matrix getX();

private:
	float dt;


private:
	Matrix x { 1, 1 };
	Matrix H { 1, 1 };
	Matrix Q { 1, 1 };
	Matrix R { 1, 1 };
	Matrix P { 1, 1 };
	Matrix I { 1, 1 };
	Matrix A { 1, 1 };


	friend class TestKFTracking;

};

#endif /* _KF_TRACKING_H_ */
