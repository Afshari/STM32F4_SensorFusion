
#ifndef _KF_PASSIVE_SUSPENSION_H_
#define _KF_PASSIVE_SUSPENSION_H_

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


#define M_PI           3.1415926


class KFPassiveSuspension {


public:
	KFPassiveSuspension();

	virtual void initialize();
	virtual void predict();
	virtual void update(const double &param);
	virtual shared_ptr<vector<double>> getX();


private:

    // Model Parameters
    const float ms = 972.2;                   // Sprung mass
    const float mu = 113.6;                   // Unsprung mass
    const float ks = 42719.6;                 // Stiffness of the suspension
    const float kt = 101115;                  // Compressibility of the tire
    const float cs = 1095;                    // Damping of the suspension
    const float ct = 14.6;                    // Damping of the pneumatic tyre
    const float dt = 0.001;                   // Stepsize

    // Time production
    const int Tf = 8;
    const int n = int( Tf / dt );

    // Process noise variance
    const float n0 = 0.1;                      // Reference spatial frequency
    const float V  = 25/3.6;                   // Vehicle forward velocity (m/s)
    const float Gqn0 = 256*1e-6;               // Road roughness coefficient

    const float Qc_f = 4 * M_PI * Gqn0 * pow(n0, 2) * V;


#ifdef USE_CMSIS_DSP

	float32_t init_11[1*1] = { 	0 };

	float32_t init_21[2*1] = { 	0,
								0, };

	float32_t init_22[2*2] = { 	0, 0,
								0, 0  };

	float32_t init_41[4*1] = { 	0,
								0,
								0,
								0, };

	float32_t init_42[4*2] = { 	0, 0,
								0, 0,
								0, 0,
								0, 0  };

	float32_t init_44[4*4] = { 	0, 0, 0, 0,
								0, 0, 0, 0,
								0, 0, 0, 0,
								0, 0, 0, 0  };

    arm_matrix_instance_f32 x;
    arm_matrix_instance_f32 P;
    arm_matrix_instance_f32 F;
    arm_matrix_instance_f32 H;
    arm_matrix_instance_f32 Q;
    arm_matrix_instance_f32 Qc;
    arm_matrix_instance_f32 R;
    arm_matrix_instance_f32 _I;


    float32_t x_f32[4*1];
    float32_t P_f32[4*4];
    float32_t F_f32[4*4];
    float32_t H_f32[1*4];
    float32_t Q_f32[4*4];
    float32_t R_f32[1*1];
    float32_t _I_f32[4*4];
    

#else

		vector<double>  x;
    vector<double>  P;
    vector<double>  F;
    vector<double>  H;
    vector<double>  Q;
    vector<double>  R;
    vector<double> HT;
    vector<double> _I;

#endif


    friend class TestKFPassiveSuspension;

};

#endif /* _KF_PASSIVE_SUSPENSION_H_ */
