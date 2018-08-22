#ifndef ROBOCALC_ROBOT_LTIN_STEP_H_
#define ROBOCALC_ROBOT_LTIN_STEP_H_

#include "LTin_Attributes.h"
#include "Operations.h"

class LTin_Step: public LTin_Attributes, public Operations {
public:
	LTin_Step(
);
	virtual ~LTin_Step();
	void Sensors();
	void Actuators();
};

#endif
