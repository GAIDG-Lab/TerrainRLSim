#pragma once

#include "scenarios/ScenarioImitateMocapEval.h"
#include "scenarios/ScenarioExpImitateMocapTarget.h"

class cScenarioImitateMocapTargetEval : virtual public cScenarioImitateMocapEval, virtual public cScenarioExpImitateMocapTarget
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioImitateMocapTargetEval();
	virtual ~cScenarioImitateMocapTargetEval();

	virtual std::string GetName() const;

protected:
};