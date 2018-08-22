#ifndef ROBOCALC_INTERFACE_LTIN_ATTRIBUTES_H_
#define ROBOCALC_INTERFACE_LTIN_ATTRIBUTES_H_

#include "HardwareComponent.h"
#include "DataTypes.h"

class LTin_Attributes : public HardwareComponent {
public:
	LTin_Attributes();
	virtual ~LTin_Attributes();

	virtual void Sensors();
	virtual void Actuators();

public:
	Double2D LTin_loc;
	double angle_to_move;
	double cellSpeed;
	int cell_id;
	Tracking_Stats tracking;

};


#endif
