
#ifndef _TEST_NETWORK_DATA_HANDLER_H_
#define _TEST_NETWORK_DATA_HANDLER_H_


#include <assert.h>
#include <math.h>
#include "tests/test_base.h"
#include "network_data_handler.h"

class TestNetworkDataHandler : public TestBase {
public:
	TestNetworkDataHandler();

	void init();
    void testConstructor();
    void testHasStart();
    void testHasEnd();
    void testGetDataLength();
    void testGetData();
    void testRemoveStart();
    void testRemoveEnd();
    void testProcess_1();
		void testProcess_2();


};

#endif /* _TEST_NETWORK_DATA_HANDLER_H_ */
