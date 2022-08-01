
#include <iostream>
#include <gtest/gtest.h>

#include "gmock_input_parser.h"
#include "gmock_kf_tracking.h"
#include "gmock_recursive_least_squares.h"
#include "gmock_kf_passive_suspension.h"

int main(int argc, char *argv[]) {
	
	::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
	return 0;
}