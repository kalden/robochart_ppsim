#include "LTiStep.h"

LTiStep::LTiStep(
):
		LTi_Attributes(), Operations() 
		{}

LTiStep::~LTiStep() {}


void LTiStep::Sensors() {
	LTi_Attributes::Sensors();
	Operations::Sensors();
}

void LTiStep::Actuators() {
	LTi_Attributes::Actuators();
	Operations::Actuators();
}

