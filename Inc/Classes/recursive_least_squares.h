#ifndef _RECURSIVE_LEAST_SQUARES_H_
#define _RECURSIVE_LEAST_SQUARES_H_

#include <string>
#include <cmath>
#include <algorithm>
#include <memory>
#include <vector>
#include "app.h"

#ifndef GTEST
#include "stm32f4xx_hal.h"
#endif

#ifdef USE_CMSIS_DSP
#include "arm_math.h"
#else
#include "linalg.h"
#endif

using std::string;
using std::vector;
using std::shared_ptr;
using std::make_shared;

class RecursiveLeastSquares {

public:
	RecursiveLeastSquares();

	virtual shared_ptr<vector<double>> getX();
	virtual void initialize(const vector<double> &params);
	virtual void calculate (const vector<double> &params);



#ifdef USE_CMSIS_DSP

private:

	arm_matrix_instance_f32 x;
	arm_matrix_instance_f32 H;
	arm_matrix_instance_f32 y;
	arm_matrix_instance_f32 R;
	arm_matrix_instance_f32 P;

	float32_t x_f32[2*1];
	float32_t H_f32[1*2];
	float32_t y_f32[1*1];
	float32_t R_f32[1*1];
	float32_t P_f32[2*2];

	float32_t init_11[1*1] = { 	0 };

	float32_t init_21[2*1] = { 	0,
															0,  };

	float32_t init_22[2*2] = { 	0, 0,
															0, 0  };


	float dt;



#else
								
private:
	vector<double> x;
	vector<double> R;
	vector<double> P;



#endif

	friend class TestRecursiveLeastSquares;

};

#endif /* _RECURSIVE_LEAST_SQUARES_H_ */

