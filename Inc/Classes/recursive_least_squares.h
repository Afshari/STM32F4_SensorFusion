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
#else
#include "matrix.h"
#endif

using std::string;
using std::array;
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

	array<float32_t, 2*1> x_f32;
	array<float32_t, 1*2> H_f32;
	array<float32_t, 1*1> y_f32;
	array<float32_t, 1*1> R_f32;
	array<float32_t, 2*2> P_f32;

	arm_matrix_instance_f32 S;
	arm_matrix_instance_f32 SI;
	arm_matrix_instance_f32 HP;
	arm_matrix_instance_f32 HT;
	arm_matrix_instance_f32 HPHT;
	arm_matrix_instance_f32 PHT;
	arm_matrix_instance_f32 K;
	arm_matrix_instance_f32 Hx;
	arm_matrix_instance_f32 yHx;
	arm_matrix_instance_f32 KyHx;
	arm_matrix_instance_f32 I;
	arm_matrix_instance_f32 KH;
	arm_matrix_instance_f32 IKH;

	array<float32_t, 1*1> S_f32;
	array<float32_t, 1*2> HP_f32;
	array<float32_t, 2*1> HT_f32;
	array<float32_t, 1*1> HPHT_f32;
	array<float32_t, 2*1> PHT_f32;
	array<float32_t, 2*1> K_f32;
	array<float32_t, 1*1> Hx_f32;
	array<float32_t, 1*1> yHx_f32;
	array<float32_t, 2*1> KyHx_f32;
	array<float32_t, 2*2> KH_f32;
	array<float32_t, 2*2> IKH_f32;

	float32_t init_11[1*1] = { 	0 };

	float32_t init_21[2*1] = { 	0,
															0,  };

	float32_t init_22[2*2] = { 	0, 0,
															0, 0  };


	float dt;



#else
								
private:
	Matrix x { 1, 1 };
	Matrix R { 1, 1 };
	Matrix P { 1, 1 };

#endif

	friend class TestRecursiveLeastSquares;

};

#endif /* _RECURSIVE_LEAST_SQUARES_H_ */

