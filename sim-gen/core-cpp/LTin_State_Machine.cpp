#include "LTin_State_Machine.h"  

LTin_State_Machine::LTin_State_Machine(
			std::shared_ptr<LTin_Step> R_LTin_Step, 
			std::shared_ptr<LTin_Controller> C_LTin_Controller) :R_LTin_Step(R_LTin_Step), C_LTin_Controller(C_LTin_Controller), StateMachine("LTin_State_Machine"), move_finished(false), adhered(false), contactedCell_ID(-1), probabilityOfAdhesion(0), distanceToMove(0), distanceMoved(0), movement_interval(0.1)
{
	pJunctionValue = (double)rand() / RAND_MAX;
	T = std::make_shared<robochart::Timer>("T");
	

	std::shared_ptr<LTin_State_Machine> ptr(this);
	// instantiate states && add substates of machine
	std::shared_ptr<i0> S_LTin_State_Machine_i0 = std::make_shared<i0>(R_LTin_Step, C_LTin_Controller, ptr);
	states.push_back(S_LTin_State_Machine_i0);
	std::shared_ptr<Moving> S_LTin_State_Machine_Moving = std::make_shared<Moving>(R_LTin_Step, C_LTin_Controller, ptr);
	states.push_back(S_LTin_State_Machine_Moving);
	std::shared_ptr<Adhesion_Response> S_LTin_State_Machine_Adhesion_Response = std::make_shared<Adhesion_Response>(R_LTin_Step, C_LTin_Controller, ptr);
	states.push_back(S_LTin_State_Machine_Adhesion_Response);

	std::shared_ptr<t0> S_LTin_State_Machine_t0 = std::make_shared<t0>(R_LTin_Step, C_LTin_Controller, ptr, S_LTin_State_Machine_i0, S_LTin_State_Machine_Moving);
	S_LTin_State_Machine_i0->transitions.push_back(S_LTin_State_Machine_t0);
	std::shared_ptr<t1> S_LTin_State_Machine_t1 = std::make_shared<t1>(R_LTin_Step, C_LTin_Controller, ptr, S_LTin_State_Machine_Moving, S_LTin_State_Machine_Moving);
	S_LTin_State_Machine_Moving->transitions.push_back(S_LTin_State_Machine_t1);
	std::shared_ptr<t2> S_LTin_State_Machine_t2 = std::make_shared<t2>(R_LTin_Step, C_LTin_Controller, ptr, S_LTin_State_Machine_Adhesion_Response, S_LTin_State_Machine_Adhesion_Response);
	S_LTin_State_Machine_Adhesion_Response->transitions.push_back(S_LTin_State_Machine_t2);
	std::shared_ptr<t3> S_LTin_State_Machine_t3 = std::make_shared<t3>(R_LTin_Step, C_LTin_Controller, ptr, S_LTin_State_Machine_Moving, S_LTin_State_Machine_Adhesion_Response);
	S_LTin_State_Machine_Moving->transitions.push_back(S_LTin_State_Machine_t3);
	std::shared_ptr<t4> S_LTin_State_Machine_t4 = std::make_shared<t4>(R_LTin_Step, C_LTin_Controller, ptr, S_LTin_State_Machine_Adhesion_Response, S_LTin_State_Machine_Moving);
	S_LTin_State_Machine_Adhesion_Response->transitions.push_back(S_LTin_State_Machine_t4);

	stage = s_Enter;
}

LTin_State_Machine::~LTin_State_Machine() {
}

int LTin_State_Machine::Initial() {
	return 0;
}

void LTin_State_Machine::Execute() {
	T->IncCounter();
	StateMachine::Execute();
}
