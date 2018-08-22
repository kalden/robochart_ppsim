#ifndef ROBOCALC_CONTROLLERS_LTO_CONTROLLER_H_
#define ROBOCALC_CONTROLLERS_LTO_CONTROLLER_H_

#include "LTo_Step.h"
#include "Controller.h"

class LTo_Controller: public robochart::Controller {
public:
	LTo_Controller(
			std::shared_ptr<LTo_Step> R_LTo_Step);
	virtual ~LTo_Controller();
	virtual void Execute();

private:
	std::shared_ptr<LTo_Step> R_LTo_Step;

};

#endif
