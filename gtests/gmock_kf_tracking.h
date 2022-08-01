#ifndef _GMOCK_KF_TRACKING_H_
#define _GMOCK_KF_TRACKING_H_

#include <tuple>
#include <iostream>

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <gmock/gmock-matchers.h>

#include "../Inc/Classes/kf_tracking.h"
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


class MockKFTracking : public KFTracking {

public:

    MOCK_METHOD1(initialize,       	void(const vector<double> &params));

    MOCK_METHOD0(predict,          	void());
    MOCK_METHOD1(update,           	void(Matrix& z));
    MOCK_METHOD0(getX,             	Matrix());
};

TEST(KFTracking, Code110) {

    string data = "110:79,185,0.1,0.1,0.1,1";

	shared_ptr<RecursiveLeastSquares> rls = make_shared<RecursiveLeastSquares>();
    shared_ptr<MockKFTracking> kf_tracking = make_shared<MockKFTracking>();
	shared_ptr<KFPassiveSuspension> kf_passive_suspension = make_shared<KFPassiveSuspension>();
    shared_ptr<MockInputParser> parser = make_shared<MockInputParser>();

    shared_ptr<vector<int>> indices = make_shared<vector<int>>( vector<int>( { 0, 4 } ) );
	vector<double> r_params{ 79, 185, 0.1, 0.1, 0.1, 1 };
	shared_ptr<vector<double>> params = make_shared<vector<double>>(r_params);
    Matrix x { 4, 1, { 2.2799e-317, 2.58871e-317, 3.26357e-316, 3.71619e-316 } };

    EXPECT_CALL(*parser, getCode(_))
            .Times(1)
            .WillOnce(Return(110));

    EXPECT_CALL(*parser, getIndices(_, _))
            .Times(1)
            .WillOnce(Return(indices));

    EXPECT_CALL(*parser, getDataVector(_, _, _))
            .Times(1)
            .WillOnce( Return( params ) );

    EXPECT_CALL(*kf_tracking, initialize(_))
            .Times(1);

    EXPECT_CALL(*kf_tracking, getX())
            .Times(1)
            .WillRepeatedly(Return(x));

    EXPECT_CALL(*kf_tracking, predict())
            .Times(0);

    EXPECT_CALL(*kf_tracking, update(_))
            .Times(0);

	AppHandler app_handler;
	app_handler.initialize(parser, rls, kf_tracking, kf_passive_suspension);
	string result = app_handler.processData(data);
}



TEST(KFTracking, Code111) {

    string data = "111:77,178;84,168;112,187";

    shared_ptr<RecursiveLeastSquares> rls = make_shared<RecursiveLeastSquares>();
    shared_ptr<MockKFTracking> kf_tracking = make_shared<MockKFTracking>();
	shared_ptr<KFPassiveSuspension> kf_passive_suspension = make_shared<KFPassiveSuspension>();
    shared_ptr<MockInputParser> parser = make_shared<MockInputParser>();

    shared_ptr<vector<int>> indices = make_shared<vector<int>>( vector<int>( { 0, 4 } ) );
	Matrix x { 4, 1, { 2.2799e-317, 2.58871e-317, 3.26357e-316, 3.71619e-316 } };

    shared_ptr<vector<Matrix>> measurements = make_shared<vector<Matrix>>();
    measurements->push_back(Matrix(2, 1, {77, 178}));
    measurements->push_back(Matrix(2, 1, {84, 168}));
    measurements->push_back(Matrix(2, 1, {112, 187}));

    EXPECT_CALL(*parser, getCode(_))
            .Times(1)
            .WillOnce(Return(111));

    EXPECT_CALL(*parser, getIndices(_, _))
            .Times(1)
            .WillOnce(Return(indices));

    EXPECT_CALL(*kf_tracking, initialize(_))
            .Times(0);

    EXPECT_CALL(*parser, getObservations(_, _, _))
            .Times(1)
            .WillOnce(Return(measurements));

    EXPECT_CALL(*kf_tracking, getX())
            .Times(3)
			.WillRepeatedly(Return(x));
	
    EXPECT_CALL(*kf_tracking, predict())
            .Times(3);
	
    EXPECT_CALL(*kf_tracking, update(_))
            .Times(3);


	AppHandler app_handler;
	app_handler.initialize(parser, rls, kf_tracking, kf_passive_suspension);
	string result = app_handler.processData(data);
}


#endif // _GMOCK_KF_TRACKING_H_
