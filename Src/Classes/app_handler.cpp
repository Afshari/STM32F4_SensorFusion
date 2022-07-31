
#include "app_handler.h"

AppHandler::AppHandler() {

	initialize();
}


void AppHandler::initialize() {

	input_parser = make_unique<InputParser>();
	kf_gps_tracking =	make_unique<KFTracking>();
	rls = make_unique<RecursiveLeastSquares>();
	kf_passive_suspension = make_unique<KFPassiveSuspension>();
}

string AppHandler::processData(const string &data) {

	string outputStr = "";

	int  code = input_parser->getCode(data);
	auto indices = input_parser->getIndices(data, ":");

	int start_index = indices->at(1);
	int len = data.length() - indices->at(1);
	

	if(code == 100) {
		vector<double> params = *input_parser->getDataVector(data, start_index, len);
		rls = make_unique<RecursiveLeastSquares>();
		rls->initialize(params);
		Matrix x = rls->getX();
		outputStr += std::to_string(x.at(0, 0)) + "," + std::to_string(x.at(1, 0));
		addExtra(outputStr);
	} else if(code == 101) {
		vector<double> params = *input_parser->getDataVector(data, start_index, len);
		rls->calculate(params);
		Matrix x = rls->getX();
		outputStr += std::to_string(x.at(0, 0)) + "," + std::to_string(x.at(1, 0));
		addExtra(outputStr);
		
	} else if(code == 110) {
		vector<double> params = *input_parser->getDataVector(data, start_index, len);
		kf_gps_tracking =	make_unique<KFTracking>();
		kf_gps_tracking->initialize(params);
		Matrix x = kf_gps_tracking->getX();
		outputStr += std::to_string(x.at(0, 0)) + "," + std::to_string(x.at(1, 0));
		addExtra(outputStr);
	} else if(code == 111) {
		vector<Matrix> measurements = *input_parser->getObservations(data, start_index, len);
		for(auto z : measurements) {
			kf_gps_tracking->predict();
			kf_gps_tracking->update(z);
			Matrix x = kf_gps_tracking->getX();
			if(outputStr != "")
				outputStr += ";";
			outputStr += std::to_string(x.at(0, 0)) + "," + std::to_string(x.at(1, 0));
		}
		addExtra(outputStr);
		
	} else if(code == 120) {
		vector<double> params = *input_parser->getDataVector(data, start_index, len);
		kf_passive_suspension->initialize();
		for(auto param : params) {
			kf_passive_suspension->predict();
			kf_passive_suspension->update(param);
			Matrix x = kf_passive_suspension->getX();
			if(outputStr == "")
				outputStr = std::to_string(x.at(0, 0)) + "," + std::to_string(x.at(1, 0));
			else
				outputStr += ";" + std::to_string(x.at(0, 0)) + "," + std::to_string(x.at(1, 0));
		}
	} else if(code == 121) {

		vector<double> params = *input_parser->getDataVector(data, start_index, len);
		for(auto param : params) {
			kf_passive_suspension->predict();
			kf_passive_suspension->update(param);
			Matrix x = kf_passive_suspension->getX();
			if(outputStr == "")
				outputStr = std::to_string(x.at(0, 0))+ "," + std::to_string(x.at(1, 0));
			else
				outputStr += ";" + std::to_string(x.at(0, 0)) + "," + std::to_string(x.at(1, 0));
		}
	}

	return outputStr;
}


void AppHandler::addExtra(string& str, int len) {
	
	string concat = "";
	for(int i = str.length(); i < len; i++) {
		concat += "0";
	}
	str += concat;
}







