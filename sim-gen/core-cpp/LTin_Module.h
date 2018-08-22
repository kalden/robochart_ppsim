#ifndef ROBOCALC_MODULE_LTIN_MODULE_H_
#define ROBOCALC_MODULE_LTIN_MODULE_H_

#include "LTin_Controller.h"
#include "LTin_Step.h"

class LTin_Module {
public:
	LTin_Module() :
			LTin_Module_LTin_Controller(nullptr),
			LTin_Module_LTin_Step(nullptr) {}
	virtual ~LTin_Module() {}

	void Init();
	void Execute();

private:
	std::shared_ptr<LTin_Controller> LTin_Module_LTin_Controller;
	std::shared_ptr<LTin_Step> LTin_Module_LTin_Step;
};

#endif
