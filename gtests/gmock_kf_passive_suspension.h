#ifndef _GMOCK_KF_PASSIVE_SUSPENSION_H_
#define _GMOCK_KF_PASSIVE_SUSPENSION_H_

#include <tuple>
#include <iostream>

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <gmock/gmock-matchers.h>

#include "../Inc/Classes/kf_passive_suspension.h"
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


class MockKFPassiveSuspension : public KFPassiveSuspension {

public:

    MOCK_METHOD0(initialize,       	void());
	MOCK_METHOD0(predict,			void());
    MOCK_METHOD1(update,        	void(const double &param));
    MOCK_METHOD0(getX,             	Matrix());	
};


TEST(KFPassiveSuspension, Code120) {

	string data = "120:32,39,28,33";
	
	shared_ptr<RecursiveLeastSquares> rls = make_shared<RecursiveLeastSquares>();
    shared_ptr<KFTracking> kf_tracking = make_shared<KFTracking>();
	shared_ptr<MockKFPassiveSuspension> kf_passive_suspension = make_shared<MockKFPassiveSuspension>();
    shared_ptr<MockInputParser> parser = make_shared<MockInputParser>();
	
	shared_ptr<vector<int>> indices = make_shared<vector<int>>( vector<int>( { 0, 4 } ) );
	vector<double> r_params{ 32, 39, 28, 33 };
	shared_ptr<vector<double>> params = make_shared<vector<double>>(r_params);
    Matrix x { 2, 1, { 0, 0 } };

    EXPECT_CALL(*parser, getCode(_))
            .Times(1)
            .WillOnce(Return(120));

    EXPECT_CALL(*parser, getIndices(_, _))
            .Times(1)
            .WillOnce(Return(indices));

    EXPECT_CALL(*parser, getDataVector(_, _, _))
            .Times(1)
            .WillOnce(Return(params));
			
	EXPECT_CALL(*kf_passive_suspension, initialize())
            .Times(1);

    EXPECT_CALL(*kf_passive_suspension, getX())
            .Times(4)
            .WillRepeatedly(Return(x));

	EXPECT_CALL(*kf_passive_suspension, predict())
            .Times(4);

    EXPECT_CALL(*kf_passive_suspension, update(_))
            .Times(4);
			
	AppHandler app_handler;
	app_handler.initialize(parser, rls, kf_tracking, kf_passive_suspension);
	string result = app_handler.processData(data);
}

TEST(KFPassiveSuspension, Code121) {

	string data = "121:32,39,28,33";
	
	shared_ptr<RecursiveLeastSquares> rls = make_shared<RecursiveLeastSquares>();
    shared_ptr<KFTracking> kf_tracking = make_shared<KFTracking>();
	shared_ptr<MockKFPassiveSuspension> kf_passive_suspension = make_shared<MockKFPassiveSuspension>();
    shared_ptr<MockInputParser> parser = make_shared<MockInputParser>();
	
	shared_ptr<vector<int>> indices = make_shared<vector<int>>( vector<int>( { 0, 4 } ) );
	vector<double> r_params{ 32, 39, 28, 33 };
	shared_ptr<vector<double>> params = make_shared<vector<double>>(r_params);
    Matrix x { 2, 1, { 0, 0 } };

    EXPECT_CALL(*parser, getCode(_))
            .Times(1)
            .WillOnce(Return(121));

    EXPECT_CALL(*parser, getIndices(_, _))
            .Times(1)
            .WillOnce(Return(indices));

    EXPECT_CALL(*parser, getDataVector(_, _, _))
            .Times(1)
            .WillOnce(Return(params));
			
	EXPECT_CALL(*kf_passive_suspension, initialize())
            .Times(0);

    EXPECT_CALL(*kf_passive_suspension, getX())
            .Times(4)
            .WillRepeatedly(Return(x));

	EXPECT_CALL(*kf_passive_suspension, predict())
            .Times(4);

    EXPECT_CALL(*kf_passive_suspension, update(_))
            .Times(4);
			
	AppHandler app_handler;
	app_handler.initialize(parser, rls, kf_tracking, kf_passive_suspension);
	string result = app_handler.processData(data);
}



#endif // _GMOCK_KF_PASSIVE_SUSPENSION_H_

