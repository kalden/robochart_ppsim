#include "LTin_Controller.h"

LTin_Controller::LTin_Controller(
		std::shared_ptr<LTin_Step> R_LTin_Step): R_LTin_Step(R_LTin_Step) {}

LTin_Controller::~LTin_Controller() {
}

void LTin_Controller::Execute() {
	Controller::Execute();
}
