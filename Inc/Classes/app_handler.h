
#ifndef _APP_HANDLER_H_
#define _APP_HANDLER_H_

#include <string>
#include <memory>
#include "stm32f4xx_hal.h"

#include "app.h"
#include "input_parser.h"
#include "kf_tracking.h"
#include "recursive_least_squares.h"
#include "kf_passive_suspension.h"

using std::unique_ptr;
using std::make_unique;
using std::tuple;

class AppHandler {
public:
	AppHandler();
	void initialize();
	string processData(const string &data);


protected:
	unique_ptr<InputParser>   					input_parser;
	unique_ptr<KFTracking> 							kf_gps_tracking;
	unique_ptr<RecursiveLeastSquares> 	rls;
	unique_ptr<KFPassiveSuspension> 		kf_passive_suspension;

};

#endif /* _APP_HANDLER_H_ */








