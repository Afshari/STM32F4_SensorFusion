
#include "Tests/tests_runner.h"


TestsRunner::TestsRunner() {

}


void TestsRunner::run() {

	TestInputParser test_input_parser;
	test_input_parser.init();

	TestNetworkDataHandler test_data_network_handler;
	test_data_network_handler.init();

	TestLinearAlgebra test_linear_algebra;
	test_linear_algebra.init();

	
	while(1);

}
