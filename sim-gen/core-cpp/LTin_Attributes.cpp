#include "LTin_Attributes.h"

LTin_Attributes::LTin_Attributes
			(): 
		 	LTin_loc(Double2D()), angle_to_move(0), cellSpeed(0), cell_id(0), tracking(Tracking_Stats())
			{
				//printf("Initialising LTin_Attributes\n");
			}

LTin_Attributes::~LTin_Attributes() {
}

void LTin_Attributes::Sensors() {
}

void LTin_Attributes::Actuators() {
}

