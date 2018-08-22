#ifndef ROBOCALC_CONTROLLERS_LTIN_CONTROLLER_H_
#define ROBOCALC_CONTROLLERS_LTIN_CONTROLLER_H_

#include "LTin_Step.h"
#include "Controller.h"

class LTin_Controller: public robochart::Controller {
public:
	LTin_Controller(
			std::shared_ptr<LTin_Step> R_LTin_Step);
	virtual ~LTin_Controller();
	virtual void Execute();

private:
	std::shared_ptr<LTin_Step> R_LTin_Step;

};

#endif
