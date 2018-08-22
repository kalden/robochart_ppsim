#ifndef ROBOCALC_INTERFACE_OPERATIONS_H_
#define ROBOCALC_INTERFACE_OPERATIONS_H_

#include "HardwareComponent.h"
#include "DataTypes.h"

class Operations : public HardwareComponent {
public:
	Operations();
	virtual ~Operations();
	virtual void update_tracking_stats(Tracking_Stats tracking, int cell_id, Double2D loc, double distanceMoved);
	//false
	virtual void setPositionOnTract(int cell_id, Double2D cell_loc);
	//false
	virtual void stop_cell_on_schedule(int cell_id);
	//false
	virtual void divide_cells(Int2D gridLoc, double adhesionExpressed, double chemokineExpressed, bool maxExpressionReached, int distance, std::string entry_state);
	//false

	virtual void Sensors();
	virtual void Actuators();


};


#endif
