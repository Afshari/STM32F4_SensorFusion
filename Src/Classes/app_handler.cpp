
#include "app_handler.h"

AppHandler::AppHandler() {

	initialize();
}


void AppHandler::initialize() {


}

string AppHandler::processData(const string &data) {

	string outputStr = "";

	int  code = input_parser->getCode(data);
	auto indices = input_parser->getIndices(data, ":");

	int start_index = indices->at(1);
	int len = data.length() - indices->at(1);
	vector<double> params = *input_parser->getDataVector(data, start_index, len);

	if(code == 100) {
		kf_gps_tracking->initialize(params);
//		vector<double> x = *kf_gps_tracking->getX();
//		outputStr += std::to_string(x[0]) + "," + std::to_string(x[1]);
	} else if(code == 101) {
		kf_gps_tracking->predict();
		kf_gps_tracking->update(params);
//		vector<double> x = *kf_gps_tracking->getX();
//		outputStr += std::to_string(x[0]) + "," + std::to_string(x[1]);
	} else if(code == 110) {
		rls->initialize(params);
//		vector<double> x = *rls->getX();
//		outputStr += std::to_string(x[0]) + "," + std::to_string(x[1]);
	} else if(code == 111) {
		rls->calculate(params);
//		vector<double> x = *rls->getX();
//		outputStr += std::to_string(x[0]) + "," + std::to_string(x[1]);

	} else if(code == 120) {

		kf_passive_suspension->initialize();
		for(auto param : params) {
			kf_passive_suspension->predict();
			kf_passive_suspension->update(param);
//			vector<double> x = *kf_passive_suspension->getX();
//			if(outputStr == "")
//				outputStr = std::to_string(x[0]);
//			else
//				outputStr += ";" + std::to_string(x[0]);
		}
	} else if(code == 121) {

		for(auto param : params) {
			kf_passive_suspension->predict();
			kf_passive_suspension->update(param);
//			vector<double> x = *kf_passive_suspension->getX();
//			if(outputStr == "")
//				outputStr = std::to_string(x[0]);
//			else
//				outputStr += ";" + std::to_string(x[0]);
		}
	}

	return outputStr;
}










