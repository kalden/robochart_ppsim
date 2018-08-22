#ifndef ROBOCALC_INTERFACE_LTI_ATTRIBUTES_H_
#define ROBOCALC_INTERFACE_LTI_ATTRIBUTES_H_

#include "HardwareComponent.h"
#include "DataTypes.h"

class LTi_Attributes : public HardwareComponent {
public:
	LTi_Attributes();
	virtual ~LTi_Attributes();

	virtual void Sensors();
	virtual void Actuators();

public:
	Double2D LTi_loc;
	double angle_to_move;
	int high_chemokine_grid_square;
	double cellSpeed;
	Tracking_Stats tracking;
	int cell_id;
	double totalchemoLevels;
	TreeMap chemomap;

};


#endif
