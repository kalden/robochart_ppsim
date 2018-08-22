#include "LTin_Step.h"

LTin_Step::LTin_Step(
):
		LTin_Attributes(), Operations() 
		{}

LTin_Step::~LTin_Step() {}


void LTin_Step::Sensors() {
	LTin_Attributes::Sensors();
	Operations::Sensors();
}

void LTin_Step::Actuators() {
	LTin_Attributes::Actuators();
	Operations::Actuators();
}

