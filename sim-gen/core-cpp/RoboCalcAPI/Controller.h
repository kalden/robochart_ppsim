
#ifndef ROBOCALC_CONTROLLER_H_
#define ROBOCALC_CONTROLLER_H_

#include "State.h"

namespace robochart {

class Controller {
public:
	std::shared_ptr<StateMachine> stm;
	Controller() {}
	virtual ~Controller() {}
	virtual void Execute() {
		if (stm != nullptr) stm->Execute();
	}
	virtual void Initialise() {}
};

}

#endif

