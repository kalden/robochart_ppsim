#include "LTo_Module.h"
#include "LTo_State_Machine.h"

void LTo_Module::Init() {
	LTo_Module_LTo_Controller = std::make_shared<LTo_Controller> (LTo_Module_LTo_Step);
	std::shared_ptr<LTo_State_Machine> LTo_Controller_LTo_State_Machine = std::make_shared<LTo_State_Machine>(LTo_Module_LTo_Step, LTo_Module_LTo_Controller);
	LTo_Module_LTo_Controller->stm = LTo_Controller_LTo_State_Machine;
	LTo_Module_LTo_Step = std::make_shared<LTo_Step> ();
}

void LTo_Module::Execute() {
	LTo_Module_LTo_Step->Sensors();
	LTo_Module_LTo_Controller->Execute();
	LTo_Module_LTo_Step->Actuators();
}

