
#ifndef _APP_HANDLER_H_
#define _APP_HANDLER_H_

#include <string>
#include <memory>
#include "stm32f4xx_hal.h"

#include "app.h"
#include "input_parser.h"

using std::unique_ptr;
using std::make_unique;
using std::tuple;

class AppHandler {
public:
	AppHandler();
	void initialize();
	string processData(const string &data);


protected:

};

#endif /* _APP_HANDLER_H_ */








