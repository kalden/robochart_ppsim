#include "LTi_Attributes.h"

LTi_Attributes::LTi_Attributes
			(): 
		 	LTi_loc(Double2D()), angle_to_move(0), high_chemokine_grid_square(99), cellSpeed(0), tracking(Tracking_Stats()), cell_id(0), totalchemoLevels(0), chemomap(TreeMap())
			{
				//printf("Initialising LTi_Attributes\n");
			}

LTi_Attributes::~LTi_Attributes() {
}

void LTi_Attributes::Sensors() {
}

void LTi_Attributes::Actuators() {
}

