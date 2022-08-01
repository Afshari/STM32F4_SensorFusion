#ifndef _GMOCK_INPUT_PARSER_H_
#define _GMOCK_INPUT_PARSER_H_

#include <tuple>

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <gmock/gmock-matchers.h>


#include "../Inc/Classes/input_parser.h"


using std::unique_ptr;
using std::tuple;

using ::testing::_;
using testing::Eq;
using testing::Return;
using testing::ByMove;
using testing::AtLeast;


class MockInputParser : public InputParser {

public:

    MOCK_METHOD1(getCode,                          	int(const string& data));
    MOCK_METHOD2(getIndices,                       	shared_ptr<vector<int>>(const string& data, const string& delimiter));
	MOCK_METHOD3(getDataVector,						shared_ptr<vector<double>>(const string& data, int start_index, int len));
	MOCK_METHOD1(getVector4d,						shared_ptr<vector<double>>(const string& data));
	MOCK_METHOD3(getObservations,					shared_ptr<vector<Matrix>>(const string& data, int start_index, int len));	
};




#endif // _GMOCK_INPUT_PARSER_H_
