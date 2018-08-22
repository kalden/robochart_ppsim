#include "LTiController.h"

LTiController::LTiController(
		std::shared_ptr<LTiStep> R_LTiStep): R_LTiStep(R_LTiStep) {}

LTiController::~LTiController() {
}

void LTiController::Execute() {
	Controller::Execute();
}
