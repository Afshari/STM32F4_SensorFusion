
#ifndef _NETWORK_DATA_HANDLER_H_
#define _NETWORK_DATA_HANDLER_H_

#include <string>
#include <memory>

#include "input_parser.h"

using std::string;

enum class NetworkDataStatus {

    UnKnown, Corrupted, Complete, NotComplete
};

class NetworkDataHandler {
public:
	NetworkDataHandler();

    NetworkDataStatus process(const string &str);
    string getProcessedData();

protected:
    string curr_data;
    string last_data;
    uint16_t    data_len;
    InputParser parser;

    void reset();
    bool hasStart(const string &str);
    bool hasEnd(const string &str);
    uint16_t getDataLength(const string &str);
    string getData(const string &str);
    void setDataLen(uint16_t len);
    void removeStart(string &str);
    void removeEnd(string &str);

    friend class TestNetworkDataHandler;

};

#endif /* _NETWORK_DATA_HANDLER_H_ */
