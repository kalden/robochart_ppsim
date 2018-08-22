#include "LTo_Controller.h"

LTo_Controller::LTo_Controller(
		std::shared_ptr<LTo_Step> R_LTo_Step): R_LTo_Step(R_LTo_Step) {}

LTo_Controller::~LTo_Controller() {
}

void LTo_Controller::Execute() {
	Controller::Execute();
}
