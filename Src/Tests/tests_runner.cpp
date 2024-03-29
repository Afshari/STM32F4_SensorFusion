
#include "Tests/tests_runner.h"


TestsRunner::TestsRunner() {

}


void TestsRunner::run() {

	TestInputParser test_input_parser;
	test_input_parser.init();

	TestNetworkDataHandler test_data_network_handler;
	test_data_network_handler.init();

	TestMatrix test_matrix;
	test_matrix.init();

	TestRecursiveLeastSquares test_recursive_least_squares;
	test_recursive_least_squares.init();
	
	TestKFTracking test_kf_tracking;
	test_kf_tracking.init();

	TestKFPassiveSuspension test_kf_passive_suspnsion;
	test_kf_passive_suspnsion.init();
	
	while(1);

}
