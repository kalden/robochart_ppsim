#include "LTiModule.h"
#include "LTi_State_Machine.h"

void LTiModule::Init() {
	LTiModule_LTiController = std::make_shared<LTiController> (LTiModule_LTiStep);
	std::shared_ptr<LTi_State_Machine> LTiController_LTi_State_Machine = std::make_shared<LTi_State_Machine>(LTiModule_LTiStep, LTiModule_LTiController);
	LTiModule_LTiController->stm = LTiController_LTi_State_Machine;
	LTiModule_LTiStep = std::make_shared<LTiStep> ();
}

void LTiModule::Execute() {
	LTiModule_LTiStep->Sensors();
	LTiModule_LTiController->Execute();
	LTiModule_LTiStep->Actuators();
}

