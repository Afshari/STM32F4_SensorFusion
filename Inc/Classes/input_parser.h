
#ifndef _INPUT_PARSER_H_
#define _INPUT_PARSER_H_

#include <string.h>
#include <string>
#include <memory>
#include <vector>

#include "app.h"
#include "matrix.h"

using std::shared_ptr;
using std::make_shared;
using std::vector;
using std::string;

class InputParser {


public:
	virtual int getCode(const uint8_t params[]);
	virtual int getCode(const string &data);
	virtual bool trim(string &data);
	virtual shared_ptr<vector<int>> getIndices(const string& data, const string& delimiter);
	virtual shared_ptr<vector<double>> getDataVector(const string& data, int start_index, int len);
	virtual shared_ptr<vector<double>> getVector4d(const string &data);
	virtual shared_ptr<vector<Matrix>> getObservations(const string& data, int start_index, int len);

};

#endif /* _INPUT_PARSER_H_ */
