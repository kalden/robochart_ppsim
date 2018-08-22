#ifndef ROBOCALC_MODULE_LTO_MODULE_H_
#define ROBOCALC_MODULE_LTO_MODULE_H_

#include "LTo_Controller.h"
#include "LTo_Step.h"

class LTo_Module {
public:
	LTo_Module() :
			LTo_Module_LTo_Controller(nullptr),
			LTo_Module_LTo_Step(nullptr) {}
	virtual ~LTo_Module() {}

	void Init();
	void Execute();

private:
	std::shared_ptr<LTo_Controller> LTo_Module_LTo_Controller;
	std::shared_ptr<LTo_Step> LTo_Module_LTo_Step;
};

#endif
