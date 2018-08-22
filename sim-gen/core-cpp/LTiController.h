#ifndef ROBOCALC_CONTROLLERS_LTICONTROLLER_H_
#define ROBOCALC_CONTROLLERS_LTICONTROLLER_H_

#include "LTiStep.h"
#include "Controller.h"

class LTiController: public robochart::Controller {
public:
	LTiController(
			std::shared_ptr<LTiStep> R_LTiStep);
	virtual ~LTiController();
	virtual void Execute();

private:
	std::shared_ptr<LTiStep> R_LTiStep;

};

#endif
