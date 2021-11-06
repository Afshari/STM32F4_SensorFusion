#ifndef _KF_TRACKING_H_
#define _KF_TRACKING_H_

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
using std::shared_ptr;
using std::make_shared;
using std::vector;

class KFTracking {

public:
	KFTracking();

	virtual void initialize(const vector<double> &params);
	virtual void predict();
	virtual void update(const vector<double> &z);
	virtual shared_ptr<vector<double>> getX();

private:
	float dt;


#ifdef USE_CMSIS_DSP

private:

	arm_matrix_instance_f32 x;
	arm_matrix_instance_f32 A;
	arm_matrix_instance_f32 H;
	arm_matrix_instance_f32 Q;
	arm_matrix_instance_f32 R;
	arm_matrix_instance_f32 P;

	float32_t x_f32[4 * 1];
	float32_t A_f32[4 * 4];
	float32_t H_f32[2 * 4];
	float32_t Q_f32[4 * 4];
	float32_t R_f32[2 * 2];
	float32_t P_f32[4 * 4];

	float32_t init_21[2 * 1] = { 0, 0, };

	float32_t init_22[2 * 2] = { 	0, 0,
																0, 0 };

	float32_t init_41[4 * 1] = { 0,
															 0,
															 0,
															 0, };

	float32_t init_42[4 * 2] = { 	0, 0, 0, 0,
																0, 0, 0, 0 };

	float32_t init_44[4 * 4] = { 0, 0, 0, 0,
															 0, 0, 0, 0,
															 0, 0, 0, 0,
															 0, 0, 0, 0 };


#else

private:
	vector<double> x;
	vector<double> H;
	vector<double> Q;
	vector<double> R;
	vector<double> P;
	vector<double> I;
	vector<double> A;

#endif


	friend class TestKFTracking;

};

#endif /* _KF_TRACKING_H_ */
