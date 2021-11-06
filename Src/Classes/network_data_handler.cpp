
#include <network_data_handler.h>

NetworkDataHandler::NetworkDataHandler() {

	reset();
}

void NetworkDataHandler::reset() {

	curr_data = "";
	data_len = 0;
}

bool NetworkDataHandler::hasStart(const string &str) {
	return str[0] == 'S';
}

bool NetworkDataHandler::hasEnd(const string &str) {
	return str[str.length() - 1] == 'E';
}

void NetworkDataHandler::removeStart(string &str) {
	str = str.substr(1, str.length() - 1);
}

void NetworkDataHandler::removeEnd(string &str) {
	str = str.substr(0, str.length() - 1);
}


void NetworkDataHandler::setDataLen(uint16_t len) {

	data_len += len;
}

uint16_t NetworkDataHandler::getDataLength(const string &str) {

	auto indices = parser.getIndices( str, ":" );

	int start_index = indices->at(0);
	int len = indices->at(1) - indices->at(0) - 1;

	string strLen = str.substr(start_index, len);
	if(hasStart(strLen))
		removeStart(strLen);

	return (uint16_t) std::stoi(strLen);
}

string NetworkDataHandler::getData(const string &str) {

	auto indices = parser.getIndices( str, ":" );

	int start_index = indices->at(1);
	int len = str.length() - indices->at(1);

	string data = str.substr(start_index, len);

	return data;
}

string NetworkDataHandler::getProcessedData() {
	return  last_data;
}

NetworkDataStatus NetworkDataHandler::process(const string &_str) {

	string str = _str;
	NetworkDataStatus result = NetworkDataStatus::UnKnown;
	
	if(data_len == 0) {

		if(hasStart(str) && hasEnd(str)) {

			removeStart(str);
			removeEnd(str);
			uint16_t dataLength = getDataLength(str);
			string data = getData(str);
			if(data.length() == dataLength) {
				result = NetworkDataStatus::Complete;
				last_data = data;
				reset();
			} else {
				result = NetworkDataStatus::Corrupted;
				reset();
			}

		} else if(hasStart(str)) {

			removeStart(str);
			curr_data = getData(str);
			data_len  = getDataLength(str);
			result = NetworkDataStatus::NotComplete;

		} else if(hasEnd(str)) {

			result = NetworkDataStatus::Corrupted;
			reset();
		}
	} else {

		if(hasEnd(str)) {
			removeEnd(str);
			curr_data += str;
			if(curr_data.length() == data_len) {
				result = NetworkDataStatus::Complete;
				last_data = curr_data;
				reset();
			} else {
				result = NetworkDataStatus::Corrupted;
				reset();
			}
		} else {
			result = NetworkDataStatus::Corrupted;
			reset();
		}
	}

	return  result;
}

