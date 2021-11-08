

#ifndef _TEST_KF_PASSIVE_SUSPENSION_H_
#define _TEST_KF_PASSIVE_SUSPENSION_H_

#include <assert.h>
#include <math.h>
#include "Tests/test_base.h"
#include "kf_passive_suspension.h"


class TestKFPassiveSuspension : public TestBase {

public:
	TestKFPassiveSuspension();

	void init();
	void testInitialize();
	void testPredict();
	void testUpdate();
	void testRun();

};

#endif /* _TEST_KF_PASSIVE_SUSPENSION_H_ */
