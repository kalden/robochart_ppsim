#include "LTo_Attributes.h"

LTo_Attributes::LTo_Attributes
			(): 
		 	new_LTin_binding(false), new_LTi_binding(false), adhesionExpressed(0), chemokineExpressed(0), LTo_loc(Double2D()), expressingRET(false), maxVCAMProbability(0), retLigandTime(0), maxExpressionReached(false), adhesionIncrement(0.05), chemokineDecrement(0.005), chemokineExpressionLimit(0), cellDivisionTime(0), cell_id(0), gridLoc(Int2D()), entry_state(std::string("start")), lTinContactStateChangeTimePoint(-1), lTiContactStateChangeTimePoint(-1)
			{
				//printf("Initialising LTo_Attributes\n");
			}

LTo_Attributes::~LTo_Attributes() {
}

void LTo_Attributes::Sensors() {
}

void LTo_Attributes::Actuators() {
}

