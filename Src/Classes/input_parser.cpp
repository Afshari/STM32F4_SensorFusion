
#include "input_parser.h"


int InputParser::getCode(const string& data) {

    string delimiter = ":";

    size_t pos = data.find(delimiter);
    if(pos > data.length())
        return -1;

    string token = data.substr(0, pos);
    return std::stoi(token);
}


int InputParser::getCode(const uint8_t params[]) {

	string data( (char *) params );
	return getCode(data);
}


shared_ptr<vector<int>> InputParser::getIndices(const string &data, const string &delimiter) {

	shared_ptr<vector<int>> indices = make_shared<vector<int>>();

    int pos = 0;
    indices->push_back( pos );
    while(true) {
        pos = data.find(delimiter, pos);
        if(pos == -1)
            break;
        pos += 1;
        indices->push_back( pos );
    }

    return indices;
}



bool InputParser::trim(string &data) {

	auto indices = *getIndices(data, ":");

	string strLen = data.substr(indices.at(0), indices.at(1) - indices.at(0) - 1);
	uint16_t len = std::stoi( strLen );
	string val = data.substr(indices.at(1), data.length() - indices.at(1));

	if(len > val.length())
		return false;

	data = data.substr(indices.at(1), len);
	return true;
}


shared_ptr<vector<float>> InputParser::getDataVector(const string& data, int start_index, int len) {

    string token = data.substr(start_index, len);
    auto indices = getIndices(token, ",");

    shared_ptr<vector<float>> state = make_shared<vector<float>>(); // (indices->size());
    state->resize(indices->size());

    string currState = "";
    for(auto i = 0U; i < indices->size(); i++) {

        if(i < indices->size() - 1) {
            currState = token.substr( indices->at(i), indices->at(i + 1) - indices->at(i) - 1 );
        } else {
            currState = token.substr( indices->at(i), token.length() );
        }
        float x = std::stof( currState );

        state->at(i) = x;
    }

    return state;
}

shared_ptr<vector<double>> InputParser::getVector4d(const string &data) {

    auto indices = getIndices( data, "," );

    shared_ptr<vector<double>> result = make_shared<vector<double>>(4);

    result->at(0) = std::atof( data.substr( indices->at(0), indices->at(1) - indices->at(0) - 1 ).c_str() );
    result->at(1) = std::atof( data.substr( indices->at(1), indices->at(2) - indices->at(1) - 1 ).c_str() );
    result->at(2) = std::atof( data.substr( indices->at(2), indices->at(3) - indices->at(2) - 1 ).c_str() );
    result->at(3) = std::atof( data.substr( indices->at(3), data.length()  - indices->at(3)     ).c_str() );

    return result;
}





