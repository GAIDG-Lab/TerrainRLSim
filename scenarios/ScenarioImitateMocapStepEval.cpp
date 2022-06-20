#include "ScenarioImitateMocapStepEval.h"
#include "scenarios/ScenarioExpImitateMocapStep.h"

cScenarioImitateMocapStepEval::cScenarioImitateMocapStepEval() :
	cScenarioExpImitateMocapStep(),
	cScenarioImitateMocapTargetEval()
{
	EnableTargetPos(false);
	EnableRandTargetPos(false);
}

cScenarioImitateMocapStepEval::~cScenarioImitateMocapStepEval()
{
}

std::string cScenarioImitateMocapStepEval::GetName() const
{
	return "Imitate Step Evaluation";
}

bool cScenarioImitateMocapStepEval::HasFallen() const
{
	bool fallen = cScenarioExpImitateMocapStep::HasFallen();
#if defined(ENABLE_KIN_CONTROLLER_TEST)
	fallen = false;
#endif
	return fallen;
}
