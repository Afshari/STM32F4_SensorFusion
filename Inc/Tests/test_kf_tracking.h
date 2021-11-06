#ifndef _TEST_KF_TRACKING_H_
#define _TEST_KF_TRACKING_H_

#include <assert.h>
#include "tests/test_base.h"
#include "kf_tracking.h"
#include "app.h"

using std::vector;

class TestKFTracking : public TestBase {

private:
	KFTracking kf;

public:
	TestKFTracking();

	void init();
	void testInitialize();
	void testPredict();
	void testUpdate();

};

#endif /* _TEST_KF_TRACKING_H_ */
