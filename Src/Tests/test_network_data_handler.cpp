
#include "Tests/test_network_data_handler.h"

TestNetworkDataHandler::TestNetworkDataHandler() {


}

void TestNetworkDataHandler::init() {

    testConstructor();
    testHasStart();
    testHasEnd();
    testGetDataLength();
    testGetData();
    testRemoveStart();
    testRemoveEnd();
    testProcess_1();
		testProcess_2();

	printReport("Network Data Handler");
}

void TestNetworkDataHandler::testConstructor() {

    NetworkDataHandler network_data_handler;
    checkEqual( network_data_handler.last_data == "" , "  " );
    checkEqual( network_data_handler.data_len == 0, " " );
}

void TestNetworkDataHandler::testHasStart() {

    NetworkDataHandler network_data_handler;

    checkEqual( network_data_handler.hasStart("S102:25,3") == true , "  " );
    checkEqual( network_data_handler.hasStart("102:25,3") == false , "  " );
}

void TestNetworkDataHandler::testHasEnd() {

    NetworkDataHandler network_data_handler;

    checkEqual( network_data_handler.hasEnd("S102:25,3E") == true , "  " );
    checkEqual( network_data_handler.hasEnd("S102:25,3") == false , "  " );
}


void TestNetworkDataHandler::testGetDataLength() {

    NetworkDataHandler network_data_handler;

    checkEqual( network_data_handler.getDataLength("S12:102:1112222221") == 12, " " );
    checkEqual( network_data_handler.getDataLength("S16:102:1112222221") == 16, " " );
}

void TestNetworkDataHandler::testRemoveStart() {

    NetworkDataHandler network_data_handler;

    string data = "S12:102:1112222221E";
    network_data_handler.removeStart(data);
    checkEqual( data == "12:102:1112222221E", " " );
}

void TestNetworkDataHandler::testRemoveEnd() {

    NetworkDataHandler network_data_handler;

    string data = "S12:102:1112222221E";
    network_data_handler.removeEnd(data);
    checkEqual( data == "S12:102:1112222221", " " );
}


void TestNetworkDataHandler::testGetData() {

    NetworkDataHandler network_data_handler;

    string data = "12:102:11,12,2,22,22,12,55,12,65";
    checkEqual( network_data_handler.getData(data) == "102:11,12,2,22,22,12,55,12,65", " " );
}


void TestNetworkDataHandler::testProcess_1() {

    NetworkDataHandler network_data_handler;
    string data = "S29:102:11,12,2,22,22,12,55,12,65E";
    checkEqual( network_data_handler.process(data) == NetworkDataStatus::Complete, "Process Enum 1" );
    checkEqual( network_data_handler.getProcessedData() == "102:11,12,2,22,22,12,55,12,65", "Process Data 1" );

    data = "S29:102:11,12,2,22,22,12,55E";
    checkEqual( network_data_handler.process(data) == NetworkDataStatus::Corrupted, "Process Enum 2" );


    data = "S29:102:11,12,2,22,22,12,55";
    checkEqual( network_data_handler.process(data) == NetworkDataStatus::NotComplete, "Process Enum 3-1" );
    data = ",12,65E";
    checkEqual( network_data_handler.process(data) == NetworkDataStatus::Complete, "Process Enum 3-2" );
    checkEqual( network_data_handler.getProcessedData() == "102:11,12,2,22,22,12,55,12,65", "Process Data 3" );


    data = "S29:102:11,12,2,22,22,12,55";
    checkEqual( network_data_handler.process(data) == NetworkDataStatus::NotComplete, "Process Enum 4-1" );
    data = ",12,6E";
    checkEqual( network_data_handler.process(data) == NetworkDataStatus::Corrupted, "Process Enum 4-2" );


    data = "S29:102:11,12,2,22,22,12,55";
    checkEqual( network_data_handler.process(data) == NetworkDataStatus::NotComplete, "Process Enum 5-1" );
    data = ",12,6";
    checkEqual( network_data_handler.process(data) == NetworkDataStatus::Corrupted, "Process Enum 5-2" );

}


void TestNetworkDataHandler::testProcess_2() {
	
		NetworkDataHandler network_data_handler;
	
	  string data = "S653:102:0.0000,572.2,0.0771,572.2,0.1541,572.2,0.2308,572.2,0.3072,572.2,0.3831,572.2,0.4584,572.2,0.5330,572.2,0.6068,572.2,0.6796,572.2,0.7514,572.2,0.8220,572.2,0.8914,572.2,0.9594,572.2,1.0259,572.2,1.0909,572.2,1.1541,572.2,1.2156,572.2,1.2752,572.2,1.3328,572.2,1.3884,572.2,1.4418,572.2,1.4931,572.2,1.5420,572.2,1.5885,572.2,1.6326,572.2,1.6742,572.2,1.7131,572.2,1.7495,572.2,1.7831,572.2,1.8140,572.2,1.8421,572.2,1.8674,572.2,1.8898,572.2,1.9092,572.2,1.9258,572.2,1.9393,572.2,1.9499,572.2,1.9574,572.2,1.9620,572.2,1.9635,572.2,1.9620,572.2,1.9574,572.2,1.9499,572.2,1.9393,572.2,1.9258,572.2,1.9092,572.2,1.8898,572.2,1.8674,572.2,1.8421,572.2E";
    checkEqualBool( network_data_handler.process(data) == NetworkDataStatus::Complete, "Process Enum 6" );
    checkEqualBool( network_data_handler.getProcessedData() == "102:0.0000,572.2,0.0771,572.2,0.1541,572.2,0.2308,572.2,0.3072,572.2,0.3831,572.2,0.4584,572.2,0.5330,572.2,0.6068,572.2,0.6796,572.2,0.7514,572.2,0.8220,572.2,0.8914,572.2,0.9594,572.2,1.0259,572.2,1.0909,572.2,1.1541,572.2,1.2156,572.2,1.2752,572.2,1.3328,572.2,1.3884,572.2,1.4418,572.2,1.4931,572.2,1.5420,572.2,1.5885,572.2,1.6326,572.2,1.6742,572.2,1.7131,572.2,1.7495,572.2,1.7831,572.2,1.8140,572.2,1.8421,572.2,1.8674,572.2,1.8898,572.2,1.9092,572.2,1.9258,572.2,1.9393,572.2,1.9499,572.2,1.9574,572.2,1.9620,572.2,1.9635,572.2,1.9620,572.2,1.9574,572.2,1.9499,572.2,1.9393,572.2,1.9258,572.2,1.9092,572.2,1.8898,572.2,1.8674,572.2,1.8421,572.2", "Process Data 6" );
}





