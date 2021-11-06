
#ifndef _TEST_INPUT_LINALG_H_
#define _TEST_INPUT_LINALG_H_

#include <assert.h>
#include "Tests/test_base.h"
#include "linalg.h"

class TestLinearAlgebra : public TestBase {

public:
	TestLinearAlgebra();

	void init();
	void testAdd();
	void testSub();
	void testScale();
	void testMul_1();
	void testMul_2();
	void testMul_3();
	void testMul_4();
	void testTran_1();
	void testTran_2();
	void testTran_3();

};

#endif /* _TEST_INPUT_LINALG_H_ */