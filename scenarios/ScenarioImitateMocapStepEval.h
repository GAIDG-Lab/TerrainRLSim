#pragma once

#include "scenarios/ScenarioImitateMocapTargetEval.h"
#include "scenarios/ScenarioExpImitateMocapStep.h"

class cScenarioImitateMocapStepEval : virtual public cScenarioImitateMocapTargetEval, virtual public cScenarioExpImitateMocapStep
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioImitateMocapStepEval();
	virtual ~cScenarioImitateMocapStepEval();

	virtual std::string GetName() const;

protected:

	virtual bool HasFallen() const;
};