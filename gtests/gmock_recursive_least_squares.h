#ifndef _GMOCK_RECURSIVE_LEAST_SQUARES_H_
#define _GMOCK_RECURSIVE_LEAST_SQUARES_H_

#include <tuple>
#include <iostream>

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <gmock/gmock-matchers.h>

#include "../Inc/Classes/recursive_least_squares.h"
#include "../Inc/Classes/input_parser.h"
#include "../Inc/Classes/app_handler.h"
#include "gmock_input_parser.h"

using std::unique_ptr;
using std::tuple;
using ::testing::_;
using testing::Eq;
using testing::Return;
using testing::ByMove;
using testing::AtLeast;


class MockRecursiveLeastSquares : public RecursiveLeastSquares {

public:

    MOCK_METHOD1(initialize,       	void(const vector<double> &params));
    MOCK_METHOD1(calculate,        	void(const vector<double> &params));
    MOCK_METHOD0(getX,             	Matrix());
};

TEST(RecursiveLeastSquares, Code100) {

	string data = "100:8,7,0.1000";
	
	shared_ptr<MockRecursiveLeastSquares> rls = make_shared<MockRecursiveLeastSquares>();
    shared_ptr<KFTracking> kf_tracking = make_shared<KFTracking>();
	shared_ptr<KFPassiveSuspension> kf_passive_suspension = make_shared<KFPassiveSuspension>();
    shared_ptr<MockInputParser> parser = make_shared<MockInputParser>();
	
	shared_ptr<vector<int>> indices = make_shared<vector<int>>( vector<int>( { 0, 4 } ) );
	vector<double> r_params{ 8, 7, 0.1 };
	shared_ptr<vector<double>> params = make_shared<vector<double>>(r_params);
    Matrix x { 2, 1, { 0, 0 } };

    EXPECT_CALL(*parser, getCode(_))
            .Times(1)
            .WillOnce(Return(100));

    EXPECT_CALL(*parser, getIndices(_, _))
            .Times(1)
            .WillOnce(Return(indices));

    EXPECT_CALL(*parser, getDataVector(_, _, _))
            .Times(1)
            .WillOnce(Return(params));
			
	EXPECT_CALL(*rls, initialize(_))
            .Times(1);

    EXPECT_CALL(*rls, getX())
            .Times(1)
            .WillOnce(Return(x));

    EXPECT_CALL(*rls, calculate(_))
            .Times(0);
			
	AppHandler app_handler;
	app_handler.initialize(parser, rls, kf_tracking, kf_passive_suspension);
	string result = app_handler.processData(data);
}

TEST(RecursiveLeastSquares, Code101) {

	string data = "101:1,0.34,11.75";
	
	shared_ptr<MockRecursiveLeastSquares> rls = make_shared<MockRecursiveLeastSquares>();
    shared_ptr<KFTracking> kf_tracking = make_shared<KFTracking>();
	shared_ptr<KFPassiveSuspension> kf_passive_suspension = make_shared<KFPassiveSuspension>();
    shared_ptr<MockInputParser> parser = make_shared<MockInputParser>();
	
	shared_ptr<vector<int>> indices = make_shared<vector<int>>( vector<int>( { 0, 4 } ) );
	vector<double> r_params{ 1, 0.34, 11.75 };
	shared_ptr<vector<double>> params = make_shared<vector<double>>(r_params);
    Matrix x { 2, 1, { 0, 0 } };

    EXPECT_CALL(*parser, getCode(_))
            .Times(1)
            .WillOnce(Return(101));

    EXPECT_CALL(*parser, getIndices(_, _))
            .Times(1)
            .WillOnce(Return(indices));

    EXPECT_CALL(*parser, getDataVector(_, _, _))
            .Times(1)
            .WillOnce(Return(params));
			
	EXPECT_CALL(*rls, initialize(_))
            .Times(0);

    EXPECT_CALL(*rls, getX())
            .Times(1)
            .WillOnce(Return(x));

    EXPECT_CALL(*rls, calculate(_))
            .Times(1);
			
	AppHandler app_handler;
	app_handler.initialize(parser, rls, kf_tracking, kf_passive_suspension);
	string result = app_handler.processData(data);
}


#endif // _GMOCK_RECURSIVE_LEAST_SQUARES_H_
