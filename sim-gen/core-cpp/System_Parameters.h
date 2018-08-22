#ifndef ROBOCALC_INTERFACE_SYSTEM_PARAMETERS_H_
#define ROBOCALC_INTERFACE_SYSTEM_PARAMETERS_H_

#include "HardwareComponent.h"

class System_Parameters : public HardwareComponent {
public:
	System_Parameters();
	virtual ~System_Parameters();

	virtual void Sensors();
	virtual void Actuators();

public:
	bool chemokine_in_environment;

};


#endif
