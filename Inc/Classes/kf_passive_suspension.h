
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
#endif

#include "matrix.h"

using std::string;
using std::vector;
using std::shared_ptr;
using std::make_shared;

#ifndef M_PI
#define M_PI           3.1415926
#endif

class KFPassiveSuspension {


public:
	KFPassiveSuspension();

	virtual void initialize();
	virtual void predict();
	virtual void update(const double &param);
	virtual Matrix getX();


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

		Matrix  x 	{ 1, 1 };
    Matrix  P 	{ 1, 1 };
    Matrix  F 	{ 1, 1 };
    Matrix  H 	{ 1, 1 };
    Matrix  Q 	{ 1, 1 };
    Matrix  R 	{ 1, 1 };
    Matrix  HT 	{ 1, 1 };
    Matrix  I 	{ 1, 1 };

    friend class TestKFPassiveSuspension;

};

#endif /* _KF_PASSIVE_SUSPENSION_H_ */
