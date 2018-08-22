#ifndef ROBOCALC_ROBOT_LTO_STEP_H_
#define ROBOCALC_ROBOT_LTO_STEP_H_

#include "LTo_Attributes.h"
#include "Operations.h"

class LTo_Step: public LTo_Attributes, public Operations {
public:
	LTo_Step(
);
	virtual ~LTo_Step();
	void Sensors();
	void Actuators();
};

#endif
