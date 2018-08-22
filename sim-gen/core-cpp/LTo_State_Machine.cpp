#include "LTo_State_Machine.h"  

LTo_State_Machine::LTo_State_Machine(
			std::shared_ptr<LTo_Step> R_LTo_Step, 
			std::shared_ptr<LTo_Controller> C_LTo_Controller) :R_LTo_Step(R_LTo_Step), C_LTo_Controller(C_LTo_Controller), StateMachine("LTo_State_Machine"), retLigandProbability(0), cell_division_clock(0), maxExpressionReached(false)
{
	pJunctionValue = (double)rand() / RAND_MAX;
	T = std::make_shared<robochart::Timer>("T");
	

	std::shared_ptr<LTo_State_Machine> ptr(this);
	// instantiate states && add substates of machine
	std::shared_ptr<i0> S_LTo_State_Machine_i0 = std::make_shared<i0>(R_LTo_Step, C_LTo_Controller, ptr);
	states.push_back(S_LTo_State_Machine_i0);
	std::shared_ptr<Not_Expressing_RET_Ligand> S_LTo_State_Machine_Not_Expressing_RET_Ligand = std::make_shared<Not_Expressing_RET_Ligand>(R_LTo_Step, C_LTo_Controller, ptr);
	states.push_back(S_LTo_State_Machine_Not_Expressing_RET_Ligand);
	std::shared_ptr<Expressing_RET_Ligand> S_LTo_State_Machine_Expressing_RET_Ligand = std::make_shared<Expressing_RET_Ligand>(R_LTo_Step, C_LTo_Controller, ptr);
	states.push_back(S_LTo_State_Machine_Expressing_RET_Ligand);
	std::shared_ptr<p0> S_LTo_State_Machine_p0 = std::make_shared<p0>(R_LTo_Step, C_LTo_Controller, ptr);
	states.push_back(S_LTo_State_Machine_p0);
	std::shared_ptr<Upregulate_Adhesion_Molecules> S_LTo_State_Machine_Upregulate_Adhesion_Molecules = std::make_shared<Upregulate_Adhesion_Molecules>(R_LTo_Step, C_LTo_Controller, ptr);
	states.push_back(S_LTo_State_Machine_Upregulate_Adhesion_Molecules);
	std::shared_ptr<Expressing_Chemokines> S_LTo_State_Machine_Expressing_Chemokines = std::make_shared<Expressing_Chemokines>(R_LTo_Step, C_LTo_Controller, ptr);
	states.push_back(S_LTo_State_Machine_Expressing_Chemokines);
	std::shared_ptr<Mature_LTo> S_LTo_State_Machine_Mature_LTo = std::make_shared<Mature_LTo>(R_LTo_Step, C_LTo_Controller, ptr);
	states.push_back(S_LTo_State_Machine_Mature_LTo);

	std::shared_ptr<t0> S_LTo_State_Machine_t0 = std::make_shared<t0>(R_LTo_Step, C_LTo_Controller, ptr, S_LTo_State_Machine_i0, S_LTo_State_Machine_p0);
	S_LTo_State_Machine_i0->transitions.push_back(S_LTo_State_Machine_t0);
	std::shared_ptr<t1> S_LTo_State_Machine_t1 = std::make_shared<t1>(R_LTo_Step, C_LTo_Controller, ptr, S_LTo_State_Machine_p0, S_LTo_State_Machine_Not_Expressing_RET_Ligand);
	S_LTo_State_Machine_p0->transitions.push_back(S_LTo_State_Machine_t1);
	std::shared_ptr<t2> S_LTo_State_Machine_t2 = std::make_shared<t2>(R_LTo_Step, C_LTo_Controller, ptr, S_LTo_State_Machine_p0, S_LTo_State_Machine_Expressing_RET_Ligand);
	S_LTo_State_Machine_p0->transitions.push_back(S_LTo_State_Machine_t2);
	std::shared_ptr<t3> S_LTo_State_Machine_t3 = std::make_shared<t3>(R_LTo_Step, C_LTo_Controller, ptr, S_LTo_State_Machine_Upregulate_Adhesion_Molecules, S_LTo_State_Machine_Not_Expressing_RET_Ligand);
	S_LTo_State_Machine_Upregulate_Adhesion_Molecules->transitions.push_back(S_LTo_State_Machine_t3);
	std::shared_ptr<t4> S_LTo_State_Machine_t4 = std::make_shared<t4>(R_LTo_Step, C_LTo_Controller, ptr, S_LTo_State_Machine_Expressing_RET_Ligand, S_LTo_State_Machine_Upregulate_Adhesion_Molecules);
	S_LTo_State_Machine_Expressing_RET_Ligand->transitions.push_back(S_LTo_State_Machine_t4);
	std::shared_ptr<t5> S_LTo_State_Machine_t5 = std::make_shared<t5>(R_LTo_Step, C_LTo_Controller, ptr, S_LTo_State_Machine_Upregulate_Adhesion_Molecules, S_LTo_State_Machine_Upregulate_Adhesion_Molecules);
	S_LTo_State_Machine_Upregulate_Adhesion_Molecules->transitions.push_back(S_LTo_State_Machine_t5);
	std::shared_ptr<t6> S_LTo_State_Machine_t6 = std::make_shared<t6>(R_LTo_Step, C_LTo_Controller, ptr, S_LTo_State_Machine_Upregulate_Adhesion_Molecules, S_LTo_State_Machine_Expressing_Chemokines);
	S_LTo_State_Machine_Upregulate_Adhesion_Molecules->transitions.push_back(S_LTo_State_Machine_t6);
	std::shared_ptr<t7> S_LTo_State_Machine_t7 = std::make_shared<t7>(R_LTo_Step, C_LTo_Controller, ptr, S_LTo_State_Machine_Expressing_Chemokines, S_LTo_State_Machine_Mature_LTo);
	S_LTo_State_Machine_Expressing_Chemokines->transitions.push_back(S_LTo_State_Machine_t7);
	std::shared_ptr<t8> S_LTo_State_Machine_t8 = std::make_shared<t8>(R_LTo_Step, C_LTo_Controller, ptr, S_LTo_State_Machine_Expressing_Chemokines, S_LTo_State_Machine_Not_Expressing_RET_Ligand);
	S_LTo_State_Machine_Expressing_Chemokines->transitions.push_back(S_LTo_State_Machine_t8);
	std::shared_ptr<t9> S_LTo_State_Machine_t9 = std::make_shared<t9>(R_LTo_Step, C_LTo_Controller, ptr, S_LTo_State_Machine_Mature_LTo, S_LTo_State_Machine_Not_Expressing_RET_Ligand);
	S_LTo_State_Machine_Mature_LTo->transitions.push_back(S_LTo_State_Machine_t9);
	std::shared_ptr<t10> S_LTo_State_Machine_t10 = std::make_shared<t10>(R_LTo_Step, C_LTo_Controller, ptr, S_LTo_State_Machine_Expressing_Chemokines, S_LTo_State_Machine_Expressing_Chemokines);
	S_LTo_State_Machine_Expressing_Chemokines->transitions.push_back(S_LTo_State_Machine_t10);
	std::shared_ptr<t11> S_LTo_State_Machine_t11 = std::make_shared<t11>(R_LTo_Step, C_LTo_Controller, ptr, S_LTo_State_Machine_Expressing_Chemokines, S_LTo_State_Machine_Expressing_Chemokines);
	S_LTo_State_Machine_Expressing_Chemokines->transitions.push_back(S_LTo_State_Machine_t11);
	std::shared_ptr<t12> S_LTo_State_Machine_t12 = std::make_shared<t12>(R_LTo_Step, C_LTo_Controller, ptr, S_LTo_State_Machine_Expressing_Chemokines, S_LTo_State_Machine_Expressing_Chemokines);
	S_LTo_State_Machine_Expressing_Chemokines->transitions.push_back(S_LTo_State_Machine_t12);
	std::shared_ptr<t13> S_LTo_State_Machine_t13 = std::make_shared<t13>(R_LTo_Step, C_LTo_Controller, ptr, S_LTo_State_Machine_Mature_LTo, S_LTo_State_Machine_Mature_LTo);
	S_LTo_State_Machine_Mature_LTo->transitions.push_back(S_LTo_State_Machine_t13);
	std::shared_ptr<t14> S_LTo_State_Machine_t14 = std::make_shared<t14>(R_LTo_Step, C_LTo_Controller, ptr, S_LTo_State_Machine_i0, S_LTo_State_Machine_Expressing_Chemokines);
	S_LTo_State_Machine_i0->transitions.push_back(S_LTo_State_Machine_t14);
	std::shared_ptr<t15> S_LTo_State_Machine_t15 = std::make_shared<t15>(R_LTo_Step, C_LTo_Controller, ptr, S_LTo_State_Machine_i0, S_LTo_State_Machine_Mature_LTo);
	S_LTo_State_Machine_i0->transitions.push_back(S_LTo_State_Machine_t15);

	stage = s_Enter;
}

LTo_State_Machine::~LTo_State_Machine() {
}

int LTo_State_Machine::Initial() {
	return 0;
}

void LTo_State_Machine::Execute() {
	T->IncCounter();
	StateMachine::Execute();
}
