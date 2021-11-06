
#include "Tests/test_input_parser.h"

TestInputParser::TestInputParser() : TestBase() {

}


void TestInputParser::init() {

	testGetCode();
	testGetIndices();
	testGetDataVector();


	printReport("Input Parser");
}


void TestInputParser::testGetCode() {

	int code1 = inputParser.getCode( "100:0,0,1,1,0.1" );
	int code2 = inputParser.getCode( "101:100,201" );
	string data = "102:102,200";
	int code3 = inputParser.getCode( data );

	checkEqual(code1, 100, "There is Problem in InputParser getCode() Function");
	checkEqual(code2, 101, "There is Problem in InputParser getCode() Function");
	checkEqual(code3, 102, "There is Problem in InputParser getCode() Function");
}



void TestInputParser::testGetIndices() {

	string value_1 = "100,150,2200";
	string delimiter_1 = ",";
	vector<int> indices1 = *inputParser.getIndices( value_1, delimiter_1 );

	checkEqual(indices1.at(0), 0);
	checkEqual(indices1.at(1), 4);
	checkEqual(indices1.at(2), 8);

	string value_2 = "st";
	string delimiter_2 = ":";
	indices1 = *inputParser.getIndices( value_2, delimiter_2 );
	checkEqual(indices1.size(), 1);
	checkEqual(indices1.at(0), 0);
}


void TestInputParser::testGetDataVector() {


	vector<float> values = *inputParser.getDataVector("101:100,150", 4, 7);

	checkEqual(values[0], 100, 1e-4);
	checkEqual(values[1], 150, 1e-4);
}










