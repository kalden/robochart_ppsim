#include "LTin_Module.h"
#include "LTin_State_Machine.h"

void LTin_Module::Init() {
	LTin_Module_LTin_Controller = std::make_shared<LTin_Controller> (LTin_Module_LTin_Step);
	std::shared_ptr<LTin_State_Machine> LTin_Controller_LTin_State_Machine = std::make_shared<LTin_State_Machine>(LTin_Module_LTin_Step, LTin_Module_LTin_Controller);
	LTin_Module_LTin_Controller->stm = LTin_Controller_LTin_State_Machine;
	LTin_Module_LTin_Step = std::make_shared<LTin_Step> ();
}

void LTin_Module::Execute() {
	LTin_Module_LTin_Step->Sensors();
	LTin_Module_LTin_Controller->Execute();
	LTin_Module_LTin_Step->Actuators();
}

