
#include "Tests/test_input_parser.h"
#include <math.h>

TestInputParser::TestInputParser() : TestBase() {

}


void TestInputParser::init() {

	testGetCode();
	testGetIndices();
	testGetDataVector();
	testGetObservations();

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

	vector<double> values = *inputParser.getDataVector("101:100,150", 4, 7);

	checkEqual(values[0], 100, 1e-4);
	checkEqual(values[1], 150, 1e-4);
}

void TestInputParser::testGetObservations() {
	
		InputParser inputParser;

    string data = "100:3.5292, 0.0001,0.1,1,0.3,0.1,1.5,0.1,0.1:17.858,-0.564;18.400,-0.565;18.693,-0.524;19.246,-0.561;18.227,0.705;19.060,0.657;19.922,0.628;20.770,0.614;20.300,0.524;18.239,-3.715;18.832,-3.734;19.450,-3.796;20.093,-3.795;14.659,-2.417;15.402,-2.448;16.201,-2.497;17.012,-2.524:10,15;11,15;10,16;11,16;-10,15;-10,16;-10,17;-10,18;-9,18;-10,-15;-11,-15;-12,-15;-13,-15;10,-11;10,-12;10,-13;10,-14";
    shared_ptr<vector<int>> indices = inputParser.getIndices( data, ":" );

    int start_index = (*indices)[2];
    int len = (*indices)[3] - (*indices)[2] - 1;

    shared_ptr<vector<Matrix>> observations = inputParser.getObservations(data, start_index, len);

    vector<Matrix> ref_observations;
    ref_observations.push_back( Matrix { 2, 1, { 17.858 , -0.564 } } );
    ref_observations.push_back( Matrix { 2, 1, { 18.4 , -0.565 } } );
    ref_observations.push_back( Matrix { 2, 1, { 18.693 , -0.524 } } );
    ref_observations.push_back( Matrix { 2, 1, { 19.246 , -0.561 } } );
    ref_observations.push_back( Matrix { 2, 1, { 18.227 , 0.705 } } );
    ref_observations.push_back( Matrix { 2, 1, { 19.06 , 0.657 } } );
    ref_observations.push_back( Matrix { 2, 1, { 19.922 , 0.628 } } );
    ref_observations.push_back( Matrix { 2, 1, { 20.77 , 0.614 } } );
    ref_observations.push_back( Matrix { 2, 1, { 20.3 , 0.524 } } );
    ref_observations.push_back( Matrix { 2, 1, { 18.239 , -3.715 } } );
    ref_observations.push_back( Matrix { 2, 1, { 18.832 , -3.734 } } );
    ref_observations.push_back( Matrix { 2, 1, { 19.45 , -3.796 } } );
    ref_observations.push_back( Matrix { 2, 1, { 20.093 , -3.795 } } );
    ref_observations.push_back( Matrix { 2, 1, { 14.659 , -2.417 } } );
    ref_observations.push_back( Matrix { 2, 1, { 15.402 , -2.448 } } );
    ref_observations.push_back( Matrix { 2, 1, { 16.201 , -2.497 } } );
    ref_observations.push_back( Matrix { 2, 1, { 17.012 , -2.524 } } );

    checkEqual((int) observations->size(), ref_observations.size(), "GetObservations size does not match");

    for(auto i = 0U; i < ref_observations.size(); i++) {
			checkEqual( observations->at(i).at(0, 0), ref_observations.at(i).at(0, 0), 1e-4);
			checkEqual( observations->at(i).at(1, 0), ref_observations.at(i).at(1, 0), 1e-4 );
    }
}








