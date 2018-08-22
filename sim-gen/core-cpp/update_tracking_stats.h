#ifndef ROBOCALC_OPERATIONS_UPDATE_TRACKING_STATS_H_
#define ROBOCALC_OPERATIONS_UPDATE_TRACKING_STATS_H_

#include "State.h"
#include "LTo_Step.h"
#include "Functions.h"
#include "DataTypes.h"

#define OP_DEBUG

class update_tracking_stats: public robochart::StateMachine {
	public:
		std::shared_ptr<LTo_Step> R_LTo_Step;

	public:
		Tracking_Stats tracking;
		int cell_id;
		Double2D loc;
		double distanceMoved;
		Simulation_Main ppsim;
	public:
		update_tracking_stats(
				std::shared_ptr<LTo_Step> R_LTo_Step);
		~update_tracking_stats();
		int initial();
		virtual void Execute();

	public:

	public:
};

#endif
