
#ifndef _DEBUG_SWV_H_
#define _DEBUG_SWV_H_

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <string>
#include "stm32f4xx_hal.h"

#include "app.h"

class DebugSWV {

public:
	DebugSWV();
	static void print_debug(const char *fmt, ...);
	static int write_swv(char *ptr, int len);


};

#endif /* _DEBUG_SWV_H_ */
