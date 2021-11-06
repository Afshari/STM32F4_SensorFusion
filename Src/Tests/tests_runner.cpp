
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

	TestRecursiveLeastSquares test_recursive_least_squares;
	test_recursive_least_squares.init();
	
	while(1);

}
