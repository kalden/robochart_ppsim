#ifndef ROBOCALC_HARDWARECOMPONENT_H_
#define ROBOCALC_HARDWARECOMPONENT_H_

class HardwareComponent {

public:
	HardwareComponent() {}
	virtual ~HardwareComponent() {}
	virtual void Sensors() = 0;
	virtual void Actuators() = 0;
};

#endif
