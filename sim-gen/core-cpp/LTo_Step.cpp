#include "LTo_Step.h"

LTo_Step::LTo_Step(
):
		LTo_Attributes(), Operations() 
		{}

LTo_Step::~LTo_Step() {}


void LTo_Step::Sensors() {
	LTo_Attributes::Sensors();
	Operations::Sensors();
}

void LTo_Step::Actuators() {
	LTo_Attributes::Actuators();
	Operations::Actuators();
}

