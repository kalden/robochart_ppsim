#include "LTi_State_Machine.h"  

LTi_State_Machine::LTi_State_Machine(
			std::shared_ptr<LTiStep> R_LTiStep, 
			std::shared_ptr<LTiController> C_LTiController) :R_LTiStep(R_LTiStep), C_LTiController(C_LTiController), StateMachine("LTi_State_Machine"), move_finished(false), adhered(false), contactedCell_ID(-1), probabilityOfAdhesion(0), chemokine_response_probability(0), distanceToMove(0), distanceMoved(0), movement_interval(0)
{
	pJunctionValue = (double)rand() / RAND_MAX;
	T = std::make_shared<robochart::Timer>("T");
	

	std::shared_ptr<LTi_State_Machine> ptr(this);
	// instantiate states && add substates of machine
	std::shared_ptr<i0> S_LTi_State_Machine_i0 = std::make_shared<i0>(R_LTiStep, C_LTiController, ptr);
	states.push_back(S_LTi_State_Machine_i0);
	std::shared_ptr<Chemotactic> S_LTi_State_Machine_Chemotactic = std::make_shared<Chemotactic>(R_LTiStep, C_LTiController, ptr);
	states.push_back(S_LTi_State_Machine_Chemotactic);
	std::shared_ptr<Non_Chemotactic> S_LTi_State_Machine_Non_Chemotactic = std::make_shared<Non_Chemotactic>(R_LTiStep, C_LTiController, ptr);
	states.push_back(S_LTi_State_Machine_Non_Chemotactic);
	std::shared_ptr<p0> S_LTi_State_Machine_p0 = std::make_shared<p0>(R_LTiStep, C_LTiController, ptr);
	states.push_back(S_LTi_State_Machine_p0);
	std::shared_ptr<Adhesion_Response> S_LTi_State_Machine_Adhesion_Response = std::make_shared<Adhesion_Response>(R_LTiStep, C_LTiController, ptr);
	states.push_back(S_LTi_State_Machine_Adhesion_Response);

	std::shared_ptr<t1> S_LTi_State_Machine_t1 = std::make_shared<t1>(R_LTiStep, C_LTiController, ptr, S_LTi_State_Machine_Chemotactic, S_LTi_State_Machine_p0);
	S_LTi_State_Machine_Chemotactic->transitions.push_back(S_LTi_State_Machine_t1);
	std::shared_ptr<t2> S_LTi_State_Machine_t2 = std::make_shared<t2>(R_LTiStep, C_LTiController, ptr, S_LTi_State_Machine_p0, S_LTi_State_Machine_Non_Chemotactic);
	S_LTi_State_Machine_p0->transitions.push_back(S_LTi_State_Machine_t2);
	std::shared_ptr<t3> S_LTi_State_Machine_t3 = std::make_shared<t3>(R_LTiStep, C_LTiController, ptr, S_LTi_State_Machine_p0, S_LTi_State_Machine_Chemotactic);
	S_LTi_State_Machine_p0->transitions.push_back(S_LTi_State_Machine_t3);
	std::shared_ptr<t4> S_LTi_State_Machine_t4 = std::make_shared<t4>(R_LTiStep, C_LTiController, ptr, S_LTi_State_Machine_Non_Chemotactic, S_LTi_State_Machine_p0);
	S_LTi_State_Machine_Non_Chemotactic->transitions.push_back(S_LTi_State_Machine_t4);
	std::shared_ptr<t5> S_LTi_State_Machine_t5 = std::make_shared<t5>(R_LTiStep, C_LTiController, ptr, S_LTi_State_Machine_Non_Chemotactic, S_LTi_State_Machine_Adhesion_Response);
	S_LTi_State_Machine_Non_Chemotactic->transitions.push_back(S_LTi_State_Machine_t5);
	std::shared_ptr<t6> S_LTi_State_Machine_t6 = std::make_shared<t6>(R_LTiStep, C_LTiController, ptr, S_LTi_State_Machine_Chemotactic, S_LTi_State_Machine_Adhesion_Response);
	S_LTi_State_Machine_Chemotactic->transitions.push_back(S_LTi_State_Machine_t6);
	std::shared_ptr<t0> S_LTi_State_Machine_t0 = std::make_shared<t0>(R_LTiStep, C_LTiController, ptr, S_LTi_State_Machine_i0, S_LTi_State_Machine_Chemotactic);
	S_LTi_State_Machine_i0->transitions.push_back(S_LTi_State_Machine_t0);
	std::shared_ptr<t11> S_LTi_State_Machine_t11 = std::make_shared<t11>(R_LTiStep, C_LTiController, ptr, S_LTi_State_Machine_Adhesion_Response, S_LTi_State_Machine_Adhesion_Response);
	S_LTi_State_Machine_Adhesion_Response->transitions.push_back(S_LTi_State_Machine_t11);
	std::shared_ptr<t7> S_LTi_State_Machine_t7 = std::make_shared<t7>(R_LTiStep, C_LTiController, ptr, S_LTi_State_Machine_Adhesion_Response, S_LTi_State_Machine_Chemotactic);
	S_LTi_State_Machine_Adhesion_Response->transitions.push_back(S_LTi_State_Machine_t7);

	stage = s_Enter;
}

LTi_State_Machine::~LTi_State_Machine() {
}

int LTi_State_Machine::Initial() {
	return 0;
}

void LTi_State_Machine::Execute() {
	T->IncCounter();
	StateMachine::Execute();
}
