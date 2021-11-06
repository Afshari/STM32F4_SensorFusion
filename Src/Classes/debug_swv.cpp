

#include "debug_swv.h"


DebugSWV::DebugSWV() {

}


void DebugSWV::print_debug(const char *fmt, ...) {

    char buffer[MAX_STRING_LENGTH];

    va_list args;
    va_start(args, fmt);
    int rc = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    strcat(buffer, "\r\n");

	write_swv(buffer, rc+2);
}

int DebugSWV::write_swv(char *ptr, int len) {

	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++) {
		ITM_SendChar(*ptr);
		ptr += 1;
	}

	return len;
}
