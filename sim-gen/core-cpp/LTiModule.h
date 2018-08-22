#ifndef ROBOCALC_MODULE_LTIMODULE_H_
#define ROBOCALC_MODULE_LTIMODULE_H_

#include "LTiController.h"
#include "LTiStep.h"

class LTiModule {
public:
	LTiModule() :
			LTiModule_LTiController(nullptr),
			LTiModule_LTiStep(nullptr) {}
	virtual ~LTiModule() {}

	void Init();
	void Execute();

private:
	std::shared_ptr<LTiController> LTiModule_LTiController;
	std::shared_ptr<LTiStep> LTiModule_LTiStep;
};

#endif
