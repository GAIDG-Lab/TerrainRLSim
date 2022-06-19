#include "ScenarioImitateMocapEval.h"
#include "scenarios/ScenarioExpImitateStep.h"

cScenarioImitateMocapEval::cScenarioImitateMocapEval() :
	cScenarioExpImitateStep(),
	cScenarioImitateTargetEval()
{
	EnableTargetPos(false);
	EnableRandTargetPos(false);
}

cScenarioImitateMocapEval::~cScenarioImitateMocapEval()
{
}

std::string cScenarioImitateMocapEval::GetName() const
{
	return "Imitate Step Evaluation";
}

bool cScenarioImitateMocapEval::HasFallen() const
{
	bool fallen = cScenarioExpImitateStep::HasFallen();
#if defined(ENABLE_KIN_CONTROLLER_TEST)
	fallen = false;
#endif
	return fallen;
}
