
#ifndef _TEST_INPUT_PARSER_H_
#define _TEST_INPUT_PARSER_H_

#include <assert.h>
#include "Tests/test_base.h"
#include "input_parser.h"

class TestInputParser : public TestBase {

public:
	TestInputParser();

	void init();
	void testGetCode();
	void testGetIndices();
	void testGetDataVector();

protected:
	InputParser inputParser;

};

#endif /* _TEST_INPUT_PARSER_H_ */
