#include "update_tracking_stats.h"  

update_tracking_stats::update_tracking_stats(
			std::shared_ptr<LTo_Step> R_LTo_Step) :R_LTo_Step(R_LTo_Step), StateMachine("update_tracking_stats"), tracking(Tracking_Stats()), cell_id(0), loc(Double2D()), distanceMoved(0), ppsim(Simulation_Main())
{
	std::shared_ptr<update_tracking_stats> ptr(this);
	// instantiate states && add substates of machine


	stage = s_Enter;
}

update_tracking_stats::~update_tracking_stats() {
}

int update_tracking_stats::initial() {
	return 0;
}

void update_tracking_stats::Execute() {
	StateMachine::Execute();
}
