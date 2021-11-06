
#ifndef _TEST_RECURSIVE_LEAST_SQUARES_H_
#define _TEST_RECURSIVE_LEAST_SQUARES_H_

#include <assert.h>
#include "app.h"
#include "Tests/test_base.h"
#include "recursive_least_squares.h"

class TestRecursiveLeastSquares : public TestBase  {

private:
	RecursiveLeastSquares rls;


public:
	TestRecursiveLeastSquares();

	void init();
	void testInitialize();
	void testCalculate();



};

#endif /* _TEST_RECURSIVE_LEAST_SQUARES_H_ */
