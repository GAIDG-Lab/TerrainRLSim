#pragma once

#include "OptScenarioImitateEval.h"

class cOptScenarioImitateMocapStepEval : public cOptScenarioImitateEval
{
public:
	cOptScenarioImitateMocapStepEval();
	virtual ~cOptScenarioImitateMocapStepEval();

protected:
	
	virtual void BuildScene(int id, std::shared_ptr<cScenarioPoliEval>& out_scene) const;
};