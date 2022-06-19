#pragma once

#include "scenarios/ScenarioImitateTargetEval.h"
#include "scenarios/ScenarioExpImitateStep.h"

class cScenarioImitateMocapEval : virtual public cScenarioImitateTargetEval, virtual public cScenarioExpImitateStep
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioImitateMocapEval();
	virtual ~cScenarioImitateMocapEval();

	virtual std::string GetName() const;

protected:

	virtual bool HasFallen() const;
};