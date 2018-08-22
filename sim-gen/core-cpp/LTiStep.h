#ifndef ROBOCALC_ROBOT_LTISTEP_H_
#define ROBOCALC_ROBOT_LTISTEP_H_

#include "LTi_Attributes.h"
#include "Operations.h"

class LTiStep: public LTi_Attributes, public Operations {
public:
	LTiStep(
);
	virtual ~LTiStep();
	void Sensors();
	void Actuators();
};

#endif
