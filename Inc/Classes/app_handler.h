
#ifndef _APP_HANDLER_H_
#define _APP_HANDLER_H_

#include <string>
#include <memory>

#ifndef GTEST
#include "stm32f4xx_hal.h"
#endif

#include "app.h"
#include "input_parser.h"
#include "kf_tracking.h"
#include "recursive_least_squares.h"
#include "kf_passive_suspension.h"

using std::unique_ptr;
using std::shared_ptr;
using std::make_unique;
using std::tuple;

class AppHandler {
public:
	AppHandler();
	void initialize(shared_ptr<InputParser> input_parser, shared_ptr<RecursiveLeastSquares> rls, 
									shared_ptr<KFTracking> kf_tracking, shared_ptr<KFPassiveSuspension> kf_passive_suspension);
	string processData(const string &data);


protected:
	shared_ptr<InputParser>   					input_parser;
	shared_ptr<KFTracking> 							kf_tracking;
	shared_ptr<RecursiveLeastSquares> 	rls;
	shared_ptr<KFPassiveSuspension> 		kf_passive_suspension;

	void addExtra(string& str, int len=250);

};

#endif /* _APP_HANDLER_H_ */








